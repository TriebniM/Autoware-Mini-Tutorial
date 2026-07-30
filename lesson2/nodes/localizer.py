#!/usr/bin/env python3

import math
import rospy

from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from pyproj import CRS, Transformer, Proj

from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped

class Localizer:
    def __init__(self):

        # Parameters
        self.undulation = rospy.get_param('undulation')
        utm_origin_lat = rospy.get_param('utm_origin_lat')
        utm_origin_lon = rospy.get_param('utm_origin_lon')

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)

        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
        self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)
        
        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.transform_coordinates)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
        self.br = TransformBroadcaster()

    def transform_coordinates(self, msg):
        
        #Get loc
        x, y = self.transformer.transform(msg.latitude,msg.longitude)
        x -= self.origin_x
        y -= self.origin_y
        
        #Calculate Orientation
        azimuth_correction = self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence
        yaw = self.convert_azimuth_to_yaw(math.radians(msg.azimuth - azimuth_correction))
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion(qx, qy, qz, qw)
        
        #Publish Pose
        current_pose = PoseStamped()
        current_pose.header.stamp = msg.header.stamp
        current_pose.header.frame_id = "map"
        current_pose.pose.position.x = x
        current_pose.pose.position.y = y
        current_pose.pose.position.z = msg.height - self.undulation
        current_pose.pose.orientation = orientation
        self.current_pose_pub.publish(current_pose)

        #Publish Velocity
        velocity = math.sqrt(msg.north_velocity**2 + msg.east_velocity**2)
        current_velocity = TwistStamped()
        current_velocity.header.stamp = msg.header.stamp
        current_velocity.header.frame_id = "base_link"
        current_velocity.twist.linear.x = velocity
        self.current_velocity_pub.publish(current_velocity)

        #Publish Map/Base_link transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = current_pose.pose.position.x
        t.transform.translation.y = current_pose.pose.position.y
        t.transform.translation.z = current_pose.pose.position.z
        t.transform.rotation = orientation
        self.br.sendTransform(t)


    @staticmethod
    def convert_azimuth_to_yaw(azimuth):
        """
        Converts azimuth to yaw. Azimuth is CW angle from the north. Yaw is CCW angle from the East.
        :param azimuth: azimuth in radians
        :return: yaw in radians
        """
        yaw = -azimuth + math.pi / 2
        # Clamp within 0 to 2 pi
        if yaw > 2 * math.pi:
            yaw = yaw - 2 * math.pi
        elif yaw < 0:
            yaw += 2 * math.pi

        return yaw

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('localizer')
    node = Localizer()
    node.run()