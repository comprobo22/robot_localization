""" Some convenience functions for translating between various representations
    of a robot pose. """

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from angle_helpers import euler_from_quaternion
from rclpy.time import Time
from rclpy.duration import Duration
import math
import PyKDL

def stamped_transform_to_pose(t):
    t = t.transform
    return Pose(position=Point(x=t.translation.x, y=t.translation.y, z=t.translation.z),
                orientation=Quaternion(x=t.rotation.x, y=t.rotation.y, z=t.rotation.z, w=t.rotation.w))

class TFHelper(object):
    """ TFHelper Provides functionality to convert poses between various
        forms, compare angles in a suitable way, and publish needed
        transforms to ROS """
    def __init__(self, node):
        self.logger = node.get_logger()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.tf_broadcaster = TransformBroadcaster(node)
        self.node = node        # hold onto this for logging
        self.transform_tolerance = Duration(seconds=0.08)    # tolerance for mismatch between scan and odom timestamp

    def convert_translation_rotation_to_pose(self, translation, rotation):
        """ Convert from representation of a pose as translation and rotation
            (Quaternion) tuples to a geometry_msgs/Pose message """
        return Pose(position=Point(x=translation[0],
                                   y=translation[1],
                                   z=translation[2]),
                    orientation=Quaternion(x=rotation[0],
                                           y=rotation[1],
                                           z=rotation[2],
                                           w=rotation[3]))

    def convert_pose_inverse_transform(self, pose):
        rot = PyKDL.Rotation.Quaternion(x=pose.orientation.x,
                                        y=pose.orientation.y,
                                        z=pose.orientation.z,
                                        w=pose.orientation.w)
        t = PyKDL.Vector(x=pose.position.x,
                         y=pose.position.y,
                         z=pose.position.z)
        transform = PyKDL.Frame(R=rot, V=t)
        inverse_transform = PyKDL.Frame.Inverse(transform)
        return (tuple(inverse_transform.p),
                inverse_transform.M.GetQuaternion())

    def convert_pose_to_xy_and_theta(self, pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)
        angles = euler_from_quaternion(*orientation_tuple)
        return (pose.position.x, pose.position.y, angles[2])

    def angle_normalize(self, z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    def angle_diff(self, a, b):
        """ Calculates the difference between angle a and angle b (both should
            be in radians) the difference is always based on the closest
            rotation from angle a to angle b.
            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = self.angle_normalize(a)
        b = self.angle_normalize(b)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

    def fix_map_to_odom_transform(self, robot_pose, odom_pose):
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer.

            robot_pose: should be of type geometry_msgs/msg/Pose and represent 
                the robot's position within the map
            odom_pose: should be of type geometry_msgs/msg/Pose and represent
                the robot's position within the odometry coordinate system
            timestamp: the timestamp to associate with this transform
            """
        (translation, rotation) = \
            self.convert_pose_inverse_transform(robot_pose)
        odom_pose_frame = PyKDL.Frame(V=PyKDL.Vector(x=odom_pose.position.x,
                                                     y=odom_pose.position.y,
                                                     z=odom_pose.position.z),
                                      R=PyKDL.Rotation.Quaternion(x=odom_pose.orientation.x,
                                                                  y=odom_pose.orientation.y,
                                                                  z=odom_pose.orientation.z,
                                                                  w=odom_pose.orientation.w))
        robot_pose_frame = PyKDL.Frame(V=PyKDL.Vector(x=translation[0],
                                                      y=translation[1],
                                                      z=translation[2]),
                                       R=PyKDL.Rotation.Quaternion(x=rotation[0],
                                                                   y=rotation[1],
                                                                   z=rotation[2],
                                                                   w=rotation[3]))


        odom_to_map = robot_pose_frame * PyKDL.Frame.Inverse(odom_pose_frame)
        self.translation = odom_to_map.p
        self.rotation = odom_to_map.M.GetQuaternion()

    def send_last_map_to_odom_transform(self, map_frame, odom_frame, timestamp):
        if (not hasattr(self, 'translation') or
            not hasattr(self, 'rotation')):
            return
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = map_frame
        transform.child_frame_id = odom_frame
        transform.transform.translation.x = self.translation[0]
        transform.transform.translation.y = self.translation[1]
        transform.transform.translation.z = self.translation[2]
        transform.transform.rotation.x = self.rotation[0]
        transform.transform.rotation.y = self.rotation[1]
        transform.transform.rotation.z = self.rotation[2]
        transform.transform.rotation.w = self.rotation[3]
        self.tf_broadcaster.sendTransform(transform)

    def get_matching_odom_pose(self, odom_frame, base_frame, timestamp):
        """ Find the odometry position for a given timestamp.  We want to avoid blocking, so if the transform is
            not ready, we return None.

            returns: a tuple where the first element is the stamped transform and the second element is the
                     delta in time between the requested time and the most recent transform available """
        if self.tf_buffer.can_transform(odom_frame, base_frame, timestamp):
            # we can get the pose at the exact right time
            return (stamped_transform_to_pose(self.tf_buffer.lookup_transform(odom_frame, base_frame, timestamp)), Duration(seconds=0.0))
        elif self.tf_buffer.can_transform(odom_frame,
                                          base_frame,
                                          Time()):
            most_recent = self.tf_buffer.lookup_transform(odom_frame,
                                                          base_frame,
                                                          Time())
            delta_t = Time.from_msg(timestamp) - Time.from_msg(most_recent.header.stamp)
            return (None, delta_t)
        else:
            return (None, None)
