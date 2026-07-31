#!/usr/bin/env python3

import numpy as np
import rospy
from threading import Lock

from autoware_mini.msg import Path, VehicleCommand
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

from shapely.geometry import LineString, Point
from shapely import prepare, distance
from scipy.interpolate import interp1d

class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")
        # Internal variables
        self.lock = Lock()
        self.path_linestring = None
        self.distance_to_velocity_interpolator = None

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCommand, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('path', Path, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        if len(msg.waypoints) < 2:
            path_linestring = None
            distance_to_velocity_interpolator = None
        else:
            # Convert waypoints to a shapely LineString and prepare it for spatial queries.
            path_linestring = LineString([(w.position.x, w.position.y) for w in msg.waypoints])
            prepare(path_linestring)

            # Create a distance-to-velocity interpolator for the path.

            # Collect waypoint x and y coordinates
            waypoints_xy = np.array([(w.position.x, w.position.y) for w in msg.waypoints])
            # Calculate cumulative distances between points
            distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0)**2, axis=1)))
            # Add 0 distance in the beginning
            distances = np.insert(distances, 0, 0)
            # Extract velocity values at waypoints
            velocities = np.array([w.speed for w in msg.waypoints])
            # Create interpolator (experiment with `bounds_error` and `fill_value` to create logical behavior when the ego vehicle is before the first waypoint or after the last one)
            distance_to_velocity_interpolator = interp1d(distances, velocities, kind='linear',fill_value = (velocities[0],velocities[-1]),bounds_error=False)

        with self.lock:
            self.path_linestring = path_linestring
            self.distance_to_velocity_interpolator = distance_to_velocity_interpolator

    def current_pose_callback(self, msg):
        interpolator = self.distance_to_velocity_interpolator

        if (self.path_linestring is None) or (interpolator is None):
            steering_angle = 0.0
            linear_velocity = 0.0
            linear_acceleration = -3.0
        else:
            # Calculate the ego vehicle's distance from the path start.
            current_pose = Point([msg.pose.position.x, msg.pose.position.y])
            d_ego_from_path_start = self.path_linestring.project(current_pose)

            # Calculate the steering angle using the Pure Pursuit formula.
            _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

            # Calculate lookahead point on the path
            lookahead_point = self.path_linestring.interpolate(d_ego_from_path_start + self.lookahead_distance)
            # Calculate lookahead heading
            lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y ,
                lookahead_point.x - current_pose.x)
            # Calculate alpha
            alpha = lookahead_heading - heading
            # Recalculate the actual lookahead distance (direct distance between points)
            ld = distance(current_pose, lookahead_point)
            # Calculate steering angle using the Pure Pursuit formula
            steering_angle = np.arctan((2*self.wheel_base*np.sin(alpha))/ld)
            # Calculate linear velocity '
            
            linear_velocity = float(interpolator(d_ego_from_path_start))
            linear_acceleration = 0.0

        # Create and publish a VehicleCommand message with constant steering angle and velocity for testing.
        vehicle_cmd = VehicleCommand()
        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"
        vehicle_cmd.steering_angle = steering_angle
        vehicle_cmd.speed = linear_velocity
        vehicle_cmd.acceleration = linear_acceleration
        self.vehicle_cmd_pub.publish(vehicle_cmd)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()