#!/usr/bin/env python3

import rospy
import math
import message_filters
import traceback
import shapely
import numpy as np
import threading
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify
from autoware_mini.msg import Path, LocalPath
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3


class SimpleSpeedPlanner:

    def __init__(self):

        # Parameters
        self.default_deceleration = rospy.get_param("default_deceleration")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        synchronization_queue_size = rospy.get_param("~synchronization_queue_size")
        synchronization_slop = rospy.get_param("~synchronization_slop")
        self.distance_to_car_front = rospy.get_param("distance_to_car_front")

        # Variables
        self.current_position = None
        self.current_speed = None

        # Lock for thread safety
        self.lock = threading.Lock()

        # Publishers
        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=1, tcp_nodelay=True)
        self.visualized_path_pub = rospy.Publisher('visualized_path', LocalPath, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)

        collision_points_sub = message_filters.Subscriber('collision_points', PointCloud2, tcp_nodelay=True)
        local_path_sub = message_filters.Subscriber('extracted_local_path', Path, tcp_nodelay=True)

        ts = message_filters.ApproximateTimeSynchronizer([collision_points_sub, local_path_sub], queue_size=synchronization_queue_size, slop=synchronization_slop)
        ts.registerCallback(self.collision_points_and_path_callback)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def current_velocity_callback(self, msg):
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        self.current_position = shapely.Point(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def collision_points_and_path_callback(self, collision_points_msg, local_path_msg):
        try:
            with self.lock:
                collision_points = numpify(collision_points_msg) if len(collision_points_msg.data) > 0 else np.array([])
                current_position = self.current_position
                current_speed = self.current_speed

            if current_speed is None or current_position is None:
                rospy.logwarn_throttle(3, "%s - current speed or position not received!", rospy.get_name())
                return

            if len(local_path_msg.waypoints) == 0 or len(collision_points) == 0:
                # no local path or no collision points means no alterations to the path
                self.publish_local_path(local_path_msg)
                return

            # Create local path linestring
            local_path_xyz = np.array([(wp.position.x, wp.position.y, wp.position.z) for wp in local_path_msg.waypoints])
            local_path_linestring = shapely.LineString(local_path_xyz)

            # Project collision points onto the local path to get distances
            collision_points_shapely = shapely.points(structured_to_unstructured(collision_points[['x', 'y', 'z']]))
            collision_point_distances = np.array([local_path_linestring.project(cp) for cp in collision_points_shapely])
            stopping_point_distances = (collision_point_distances - collision_points["distance_to_stop"])
            # Calculate target velocity and modify the local path waypoint speeds.
            collision_point_braking_distances = np.array(
                [cpd-self.distance_to_car_front for i,cpd in enumerate(collision_point_distances)])
            
            target_distances = np.array(
                [cpbd-collision_points[i][6] for i,cpbd in enumerate(collision_point_braking_distances)])
            
            # Calculate collision point speed.
            collision_point_path_headings = [self.get_heading_at_distance(local_path_linestring, d) for d in collision_point_distances]
            collision_point_speeds = np.array([self.project_vector_to_heading(heading, Vector3(vx, vy, vz))
                                         for heading, (vx, vy, vz) in zip(collision_point_path_headings, collision_points[['vx', 'vy', 'vz']])])
            # Modify target velocity with reaction time.
            target_distances = np.maximum(0,target_distances - self.braking_reaction_time*np.abs(collision_point_speeds))
            # Account for collision point speed in target velocity.
            approaching_speeds = np.minimum(collision_point_speeds, 0)
            calculated_target_velocities = np.maximum(0,
                approaching_speeds + np.sqrt(np.maximum(0,
                    collision_point_speeds ** 2 + 2 * self.default_deceleration * target_distances)))
            cp_min_velo = np.argmin(calculated_target_velocities)
            min_velocity = np.min(calculated_target_velocities)
            for wp in local_path_msg.waypoints:
                            wp.speed = min(min_velocity, wp.speed)

            target_object_speed = collision_point_speeds[cp_min_velo]
            collision_point_braking_distance = collision_point_braking_distances[cp_min_velo]
            collision_point_category = collision_points[cp_min_velo]["category"]
            stopping_point_distance = stopping_point_distances[cp_min_velo]


            
            # Publishing the modified local path goes 
            path = Path()
            path.header = local_path_msg.header
            path.waypoints = local_path_msg.waypoints
            self.publish_local_path(path,
                                    target_object_distance=collision_point_braking_distance,
                                    target_object_speed=target_object_speed,
                                    stopping_point_distance=stopping_point_distance,
                                    collision_point_category=collision_point_category,
                                    is_blocked=True)

        except Exception as e:
            rospy.logerr_throttle(10, "%s - Exception in callback: %s", rospy.get_name(), traceback.format_exc())

    def publish_local_path(self, path, target_object_distance=0.0, target_object_speed=0.0,
                           stopping_point_distance=0.0, collision_point_category=0, is_blocked=False):
        # Publish the path for the controller
        self.local_path_pub.publish(path)
        # Convert to the LocalPath message used by the autoware_mini visualization and dashboard nodes
        self.visualized_path_pub.publish(LocalPath(header=path.header, waypoints=path.waypoints,
                                                   target_object_distance=float(target_object_distance),
                                                   target_object_speed=float(target_object_speed),
                                                   stopping_point_distance=float(stopping_point_distance),
                                                   collision_point_category=int(collision_point_category),
                                                   is_blocked=bool(is_blocked)))

    @staticmethod
    def get_heading_at_distance(linestring, distance):
        point_after = linestring.interpolate(distance + 0.1)
        point_before = linestring.interpolate(max(0, distance - 0.1))
        return math.atan2(point_after.y - point_before.y, point_after.x - point_before.x)

    @staticmethod
    def project_vector_to_heading(heading_angle, vector):
        return vector.x * math.cos(heading_angle) + vector.y * math.sin(heading_angle)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('simple_speed_planner', log_level=rospy.INFO)
    node = SimpleSpeedPlanner()
    node.run()
