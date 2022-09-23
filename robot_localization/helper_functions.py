""" Some convenience functions for translating between various representations
    of a robot pose. """

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from angle_helpers import euler_from_quaternion
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

    def fix_map_to_odom_transform(self, robot_pose, timestamp):
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer.

            robot_pose should be of type geometry_msgs/Pose and timestamp is of
            type rospy.Time and represents the time at which the robot's pose
            corresponds.
            """
        (translation, rotation) = \
            self.convert_pose_inverse_transform(robot_pose)
        p = PoseStamped(
            pose=self.convert_translation_rotation_to_pose(translation,
                                                           rotation),
            header=Header(stamp=timestamp, frame_id='base_footprint'))
        self.odom_to_map = self.tf_buffer.transform(p,
                                                    'odom',
                                                    timeout=rclpy.duration.Duration(seconds=1.0))
        self.timestamp = timestamp
        (self.translation, self.rotation) = \
            self.convert_pose_inverse_transform(self.odom_to_map.pose)

    def send_last_map_to_odom_transform(self, map_frame, odom_frame):
        self.logger.info('sending')
        if (not hasattr(self, 'translation') or
            not hasattr(self, 'rotation') or
            not hasattr(self, 'timestamp')):
            return
        transform = TransformStamped()
        transform.header.stamp = self.timestamp
        transform.header.frame_id = map_frame
        transform.child_frame_id = odom_frame
        transform.transform.translation.x = self.translation[0]
        transform.transform.translation.y = self.translation[1]
        transform.transform.translation.z = self.translation[2]
        transform.transform.rotation.x = self.rotation[0]
        transform.transform.rotation.y = self.rotation[1]
        transform.transform.rotation.z = self.translation[2]
        transform.transform.rotation.w = self.translation[3]
        self.tf_broadcaster.sendTransform(transform)
