#!/usr/bin/env python3

""" This is the starter code for the robot localization project """

import pdb
from typing import List
import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseArray,
    Pose,
    Point,
    Quaternion,
)
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper, draw_random_sample
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler


class Particle(object):
    """Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
    """

    ANGLE_NOISE = 0.05
    ODOM_NOISE = 0.05

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """Construct a new Particle
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized"""
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """A helper function to convert a particle to a geometry_msgs/Pose message"""
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )

    def update_position(self, delta, delta_angle):
        # map_delta = np.array([delta[0], delta[1]])
        # p_heading = np.array([[np.cos(self.theta), -np.sin(self.theta)], [np.sin(self.theta), np.cos(self.theta)]])
        # particle_delta = p_heading@map_delta
        # print(map_delta)
        # print(self.theta)
        particle_delta = self.homogeneous_pose@delta
        # print(particle_delta)
        # print(delta)
        self.x = particle_delta[0][2]
        self.y = particle_delta[1][2]
        # fix angle
        self.theta += delta_angle

    def add_noise(self):
        self.x += np.random.normal(0, self.ODOM_NOISE)
        self.y += np.random.normal(0, self.ODOM_NOISE)
        self.theta += np.random.normal(0, self.ANGLE_NOISE)

    def update_weight(self, new_w):
        self.w = new_w

    def transform_scan_point_to_map(self, r, theta):
        return (
            self.homogeneous_pose @ np.array([r * np.cos(theta), r * np.sin(theta), 1])
        )[0:2]

    def transform_scan_to_map(self, r, theta):
        return (
            self.homogeneous_pose
            @ np.array(
                [
                    np.multiply(r, np.cos(theta)),
                    np.multiply(r, np.sin(theta)),
                    np.ones(len(r)),
                ]
            )
        )[0:2, :]

    @property
    def homogeneous_pose(self):
        return np.array(
            [
                [np.cos(self.theta), -np.sin(self.theta), self.x],
                [np.sin(self.theta), np.cos(self.theta), self.y],
                [0, 0, 1],
            ]
        )


class ParticleFilter(Node):
    """The class that represents a Particle Filter ROS Node
    Attributes list:
        base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
        map_frame: the name of the map coordinate frame (should be "map" in most cases)
        odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
        scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
        n_particles: the number of particles in the filter
        d_thresh: the amount of linear movement before triggering a filter update
        a_thresh: the amount of angular movement before triggering a filter update
        pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
        particle_pub: a publisher for the particle cloud
        last_scan_timestamp: this is used to keep track of the clock when using bags
        scan_to_process: the scan that our run_loop should process next
        occupancy_field: this helper class allows you to query the map for distance to closest obstacle
        transform_helper: this helps with various transform operations (abstracting away the tf2 module)
        particle_cloud: a list of particles representing a probability distribution over robot poses
        current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                               The pose is expressed as a list [x,y,theta] (where theta is the yaw)
        thread: this thread runs your main loop
    """

    def __init__(self):
        super().__init__("pf")
        self.base_frame = "base_footprint"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.scan_topic = "scan"  # the topic where we will get laser scans from

        self.n_particles = 800  # the number of particles to use
        self.d_thresh = 0.2  # the amount of linear movement before performing an update
        self.a_thresh = (
            math.pi / 6
        )  # the amount of angular movement before performing an update

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        # self.create_subscription(
        #     PoseWithCovarianceStamped, "initialpose", self.update_initial_pose, 10
        # )

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(
            PoseArray, "particlecloud", qos_profile_sensor_data
        )

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud: List[Particle] = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """This function takes care of sending out the map to odom transform"""
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(
            seconds=0.1
        )
        self.transform_helper.send_last_map_to_odom_transform(
            self.map_frame, self.odom_frame, postdated_timestamp
        )

    def loop_wrapper(self):
        """This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2"""
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """This is the main run_loop of our particle filter.  It checks to see if
        any scans are ready and to be processed and will call several helper
        functions to complete the processing.

        You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(
            self.odom_frame, self.base_frame, msg.header.stamp
        )
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return

        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(
            msg, self.base_frame
        )

        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud()
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            t1 = time.perf_counter()
            self.update_particles_with_odom()  # update based on odometry
            t2 = time.perf_counter()
            self.update_particles_with_laser(r, theta)  # update based on laser scan
            t3 = time.perf_counter()
            self.update_robot_pose()  # update robot's pose based on particles
            t4 = time.perf_counter()
            self.resample_particles()  # resample particles to focus on areas of high density
            t5 = time.perf_counter()
            print(f'1: {t2-t1}')
            print(f'2: {t3-t2}')
            print(f'3: {t4-t3}')
            print(f'4: {t5-t4}')
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return (
            math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            > self.a_thresh
        )

    def update_robot_pose(self):
        """Update the estimate of the robot's pose given the updated particles.
        There are two logical methods for this:
            (1): compute the mean pose
            (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # Updates the pose of the robot with that of the best particle (by weight)
        best_particle = max(self.particle_cloud, key=lambda p: p.w)
        self.robot_pose = self.transform_helper.convert_translation_rotation_to_pose(
            translation=[best_particle.x, best_particle.y, 0.0],
            rotation=quaternion_from_euler(yaw=best_particle.theta),
        )
        self.transform_helper.fix_map_to_odom_transform(self.robot_pose, self.odom_pose)

    def update_particles_with_odom(self):
        """Update the particles using the newly given odometry pose.
        The function computes the value delta which is a tuple (x,y,theta)
        that indicates the change in position and angle between the odometry
        when the particles were last updated and the current odometry.
        """
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            delta = np.linalg.inv([[np.cos(self.current_odom_xy_theta[2]), -np.sin(self.current_odom_xy_theta[2]), self.current_odom_xy_theta[0]], [np.sin(self.current_odom_xy_theta[2]), np.cos(self.current_odom_xy_theta[2]), self.current_odom_xy_theta[1]], [0, 0, 1]])@[[np.cos(new_odom_xy_theta[2]), -np.sin(new_odom_xy_theta[2]), new_odom_xy_theta[0]], [np.sin(new_odom_xy_theta[2]), np.cos(new_odom_xy_theta[2]), new_odom_xy_theta[1]], [0, 0, 1]]
            delta_angle = new_odom_xy_theta[2] - self.current_odom_xy_theta[2]
            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        for particle in self.particle_cloud:
            particle.update_position(delta, delta_angle)

    def resample_particles(self):
        """Resample the particles according to the new particle weights.
        The weights stored with each particle should define the probability that a particular
        particle is selected in the resampling step.  You may want to make use of the given helper
        function draw_random_sample in helper_functions.py.
        """

        # make sure the distribution is normalized
        self.normalize_particles()

        # Resample particles based on current distribution
        self.particle_cloud = draw_random_sample(
            self.particle_cloud,
            [particle.w for particle in self.particle_cloud],
            self.n_particles,
        )
        # Add noise to each particle to expand coverage range
        for particle in self.particle_cloud:
            particle.add_noise()

        # make sure distribution is normalized after resampling
        self.normalize_particles()

    def update_particles_with_laser(self, r, theta):
        """Updates the particle weights in response to the scan data
        r: the distance readings to obstacles
        theta: the angle relative to the robot frame for each corresponding reading
        """
        for particle in self.particle_cloud:
            # Project the laser scan onto each particle
            laser_scan_map_frame = particle.transform_scan_to_map(r, theta)
            print(np.shape(laser_scan_map_frame))
            # Compute the average distance to nearby obstacles for the laser scan
            avg_dist = self.occupancy_field.get_closest_obstacle_distance(laser_scan_map_frame)
            # Update the particle's weight based on the inverse average distance
            particle.update_weight(1 / avg_dist**2)

    def update_initial_pose(self, msg):
        """Callback function to handle re-initializing the particle filter based on a pose estimate.
        These pose estimates could be generated by another ROS Node or could come from the rviz GUI"""
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self):
        """Initialize the particle cloud."""
        map_bounds = self.occupancy_field.get_obstacle_bounding_box()
        while len(self.particle_cloud) < self.n_particles:
            rand_x = np.random.uniform(low=map_bounds[0][0], high=map_bounds[0][1], size=(1))[0]
            rand_y = np.random.uniform(low=map_bounds[1][0], high=map_bounds[1][1], size=(1))[0]
            rand_theta = np.random.uniform(low=0, high=2*np.pi, size=(1))[0]
            if not math.isnan(self.occupancy_field.get_closest_obstacle_distance(rand_x, rand_y)):
                self.particle_cloud.append(
                    Particle(x=rand_x, y=rand_y, w=1.0, theta=rand_theta)
                )
        self.normalize_particles()

    def normalize_particles(self):
        """Make sure the particle weights define a valid distribution (i.e. sum to 1.0)"""
        # Divide each particle's weight by the total weight of all particles
        weight_total = sum([particle.w for particle in self.particle_cloud])
        for particle in self.particle_cloud:
            particle.update_weight(particle.w / weight_total)

    def publish_particles(self, timestamp):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(
            PoseArray(
                header=Header(stamp=timestamp, frame_id=self.map_frame),
                poses=particles_conv,
            )
        )

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop
        if self.scan_to_process is None:
            self.scan_to_process = msg


def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
