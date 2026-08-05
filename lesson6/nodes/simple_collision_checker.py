#!/usr/bin/env python3

import rospy
import shapely
import math
import numpy as np
import threading
from ros_numpy import msgify
from autoware_mini.msg import Path, DetectedObjectArray, StopLineStatusArray, StopLineStatus
from sensor_msgs.msg import PointCloud2
from shapely.geometry import LineString, Point
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector

# Collision point categories: 0 = none, 1 = goal, 2 = traffic light, 3 = static obstacle, 4 = moving obstacle
DTYPE = np.dtype([
    ('x', np.float32),          # position
    ('y', np.float32),
    ('z', np.float32),
    ('vx', np.float32),         # velocity
    ('vy', np.float32),
    ('vz', np.float32),
    ('distance_to_stop', np.float32),   # safety distance before collision point
    ('deceleration_limit', np.float32), # max allowed deceleration (np.inf = no limit)
    ('category', np.int32)
])


class SimpleCollisionChecker:

    def __init__(self):

        # Parameters
        self.safety_box_width = rospy.get_param("safety_box_width")
        self.stopped_speed_limit = rospy.get_param("stopped_speed_limit")
        self.braking_safety_distance_obstacle = rospy.get_param("~braking_safety_distance_obstacle")
        self.braking_safety_distance_goal = rospy.get_param("~braking_safety_distance_goal")
        self.braking_safety_distance_stopline = rospy.get_param("~braking_safety_distance_stopline")
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Load the lanelet2 map
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        lanelet2_map = load(lanelet2_map_path, projector)

        # Extract stop lines and traffic lights from the lanelet2 map
        # {stop_line_id: linestring, ...}
        self.stop_lines = self.get_traffic_light_stop_lines(lanelet2_map)
        # {stop_line_id: {traffic_light_id: [4 corner coordinates], ...}, ...}
        self.traffic_lights = self.get_traffic_light_bboxes(lanelet2_map)

        # Variables
        self.detected_objects = None
        self.goal_point = None
        self.stopline_statuses = {}

        # Lock for thread safety
        self.lock = threading.Lock()

        # Publishers
        self.collision_points_pub = rospy.Publisher('collision_points', PointCloud2, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('extracted_local_path', Path, self.path_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/final_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('global_path', Path, self.global_path_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/detection/traffic_light_status', StopLineStatusArray, self.traffic_light_status_callback, queue_size=1, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def traffic_light_status_callback(self, msg):
        self.stopline_statuses = {status.stop_line_id: status.status for status in msg.statuses}

    @staticmethod
    def get_traffic_light_stop_lines(lanelet2_map):
        """
        Iterate over all regulatory elements with subtype traffic_light and extract the stop lines.
        :param lanelet2_map: lanelet2 map
        :return: {stop_line_id: stop line as a shapely LineString, ...}
        """
        stop_lines = {}
        for reg_el in lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                # ref_line is the stop line and there is only 1 stop line per traffic light reg_el
                stop_line = reg_el.parameters["ref_line"][0]
                stop_lines[stop_line.id] = shapely.LineString([(p.x, p.y) for p in stop_line])

        return stop_lines

    
    @staticmethod
    def get_traffic_light_bboxes(lanelet2_map):
        """
        Iterate over all regulatory elements with subtype traffic_light and extract the traffic lights
        with their corner coordinates. One stop line can be controlled by several traffic lights (poles).
        :param lanelet2_map: lanelet2 map
        :return: {stop_line_id: {traffic_light_id: [top_left, top_right, bottom_left, bottom_right], ...}, ...}
        """
        traffic_lights = {}
        for reg_el in lanelet2_map.regulatoryElementLayer:
            if reg_el.attributes["subtype"] == "traffic_light":
                stop_line_id = reg_el.parameters["ref_line"][0].id

                for tfl in reg_el.parameters["refers"]:
                    tfl_height = float(tfl.attributes["height"])
                    # corner coordinates in order: top_left, top_right, bottom_left, bottom_right
                    corners = [(tfl[0].x, tfl[0].y, tfl[0].z + tfl_height),
                               (tfl[1].x, tfl[1].y, tfl[1].z + tfl_height),
                               (tfl[0].x, tfl[0].y, tfl[0].z),
                               (tfl[1].x, tfl[1].y, tfl[1].z)]
                    traffic_lights.setdefault(stop_line_id, {})[tfl.id] = corners

        return traffic_lights

    def detected_objects_callback(self, msg):
        self.detected_objects = msg.objects

    def global_path_callback(self, msg):
        if len(msg.waypoints) > 0:
            self.goal_point = msg.waypoints[-1].position
        else:
            self.goal_point = None

    def path_callback(self, msg):
        with self.lock:
            detected_objects = self.detected_objects
            goal_point = self.goal_point

        collision_points = np.array([], dtype=DTYPE)

        if len(msg.waypoints) == 0:
            collision_points_msg = PointCloud2()
            collision_points_msg.header = msg.header
            self.collision_points_pub.publish(collision_points_msg)
            return

        # Create obstacle collision points.
        local_path_linestring = LineString([(wp.position.x,wp.position.y) for wp in msg.waypoints])
        local_path_buffer = local_path_linestring.buffer(self.safety_box_width / 2, cap_style="flat")
        shapely.prepare(local_path_buffer)

        if detected_objects is not None and len(detected_objects) > 0:
            for obj in detected_objects:
                cvxh = obj.convex_hull
                object_polygon = shapely.Polygon([(cvxh[i],cvxh[i+1]) for i in range(0,len(cvxh),3)])
                if local_path_buffer.intersects(object_polygon):
                    intersection = local_path_buffer.intersection(object_polygon)
                    intersection_points = shapely.get_coordinates(intersection)
                    # Calculate the intersection geometry and create a collision point from each
                    # of its coordinates, filling in the rest of the DTYPE fields from the object
                    # metadata.
                    for x, y in intersection_points:
                        category = 3
                        object_speed = math.sqrt(obj.velocity.x**2 + obj.velocity.y**2)
                        if object_speed > self.stopped_speed_limit:
                            category = 4
                        collision_points = np.append(collision_points, np.array([
                            (x,
                             y,
                             obj.centroid.z,
                             obj.velocity.x,
                             obj.velocity.y,
                             obj.velocity.z,
                             self.braking_safety_distance_obstacle,
                             np.inf,category
                            )], dtype=DTYPE))

        # Add goal point as collision point.
        if goal_point is not None:
            goal_point_shapely = shapely.Point(goal_point.x, goal_point.y)
            if local_path_buffer.intersects(goal_point_shapely.buffer(0.1)):
                collision_points = np.append(collision_points, np.array(
                    [(goal_point.x,
                      goal_point.y,
                      goal_point.z,
                      0,
                      0,
                      0,
                      self.braking_safety_distance_goal,
                      np.inf,
                      1
                    )], dtype=DTYPE))
                
        # Add stop line collision points for red traffic lights
        if self.stopline_statuses:
            local_path_lineString = shapely.LineString([
                                            (waypoint.position.x, waypoint.position.y)
                                              for waypoint in msg.waypoints])
            for id, stopline_status in self.stopline_statuses.items():
                if local_path_lineString.intersects(self.stop_lines[id]) and (
                     stopline_status == StopLineStatus.STATUS_STOP):
                    intersect = local_path_lineString.intersection(self.stop_lines[id])
                    intersection_points = shapely.get_coordinates(intersect)
                    for x, y in intersection_points:
                                            collision_points = np.append(collision_points, np.array([
                                                (x,
                                                 y,
                                                 0,
                                                 0,
                                                 0,
                                                 0,
                                                 self.braking_safety_distance_stopline,
                                                 np.inf,
                                                 2
                                                )], dtype=DTYPE))

        # Publish the collision points (an empty array means no collision points on the path)
        if len(collision_points) > 0:
            collision_points_msg = msgify(PointCloud2, collision_points)
        else:
            collision_points_msg = PointCloud2()
        collision_points_msg.header = msg.header
        self.collision_points_pub.publish(collision_points_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('simple_collision_checker', log_level=rospy.INFO)
    node = SimpleCollisionChecker()
    node.run()
