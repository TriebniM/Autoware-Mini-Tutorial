#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import shapely
import threading
import tf2_ros
import onnxruntime

import image_geometry
import cv_bridge

from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from tf2_geometry_msgs import do_transform_point

from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from autoware_mini.msg import Path, StopLineStatus, StopLineStatusArray
from autoware_mini.yolo import Yolo11Model

# Force plain CUDA execution instead of TensorRT to avoid engine building delays and deployment issues
_InferenceSession = onnxruntime.InferenceSession
onnxruntime.InferenceSession = lambda model_path, **kwargs: _InferenceSession(model_path, providers=['CUDAExecutionProvider'])

# The main traffic light classes of the YOLO model. The model can also detect
# arrow-specific variants (classes 4-12, e.g. green-left) - those are discarded.
CLASS_TO_STRING = {
    0: "green",
    1: "yellow",
    2: "red",
    3: "unknown"
}

CLASS_TO_COLOR = {
    0: (0, 255, 0),
    1: (255, 255, 0),
    2: (255, 0, 0),
    3: (0, 0, 0)
}

CLASS_TO_LIGHT_COLOR = {
    0: (204, 255, 153),
    1: (255, 255, 153),
    2: (255, 153, 153),
    3: (192, 192, 192)
}

CLASS_TO_TLRESULT = {
    0: StopLineStatus.STATUS_GO,      # GREEN
    1: StopLineStatus.STATUS_STOP,    # YELLOW
    2: StopLineStatus.STATUS_STOP,    # RED
    3: StopLineStatus.STATUS_UNKNOWN  # UNKNOWN
}


class YoloTrafficLightDetector:
    def __init__(self):

        # Node parameters
        onnx_path = rospy.get_param("~onnx_path")
        self.rectify_image = rospy.get_param('~rectify_image')
        self.roi_width_extent = rospy.get_param("~roi_width_extent")
        self.roi_height_extent = rospy.get_param("~roi_height_extent")
        self.min_roi_width = rospy.get_param("~min_roi_width")
        self.transform_timeout = rospy.get_param("~transform_timeout")
        self.iou_threshold = rospy.get_param("~iou_threshold")
        self.camera_delay_compensation = rospy.get_param("~camera_delay_compensation")
        # Parameters related to lanelet2 map loading
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

        self.bridge = cv_bridge.CvBridge()
        self.yolo_model = Yolo11Model(onnx_path)

        # Variables
        self.camera_model = None
        self.stop_line_ids_on_path = None
        self.transform_from_frame = None

        # Lock for thread safety
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers
        self.tfl_status_pub = rospy.Publisher('traffic_light_status', StopLineStatusArray, queue_size=1, tcp_nodelay=True)
        self.tfl_roi_pub = rospy.Publisher('traffic_light_roi', Image, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('camera_info', CameraInfo, self.camera_info_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/planning/local_path', Path, self.local_path_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)
        rospy.Subscriber('image_raw', Image, self.camera_image_callback, queue_size=1, buff_size=2**26, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def camera_info_callback(self, camera_info_msg):
        if self.camera_model is None:
            camera_model = image_geometry.PinholeCameraModel()
            camera_model.fromCameraInfo(camera_info_msg)
            self.camera_model = camera_model

    def local_path_callback(self, local_path_msg):

        # used in calculate_roi_coordinates to filter out only relevant traffic lights
        stop_line_ids_on_path = []

        # Find stop lines along path
        if len(local_path_msg.waypoints) > 0:
            local_path_lineString = shapely.LineString([
                (waypoint.position.x, waypoint.position.y) for waypoint in local_path_msg.waypoints])
            for id,stop_line in self.stop_lines.items():
                if local_path_lineString.intersects(stop_line):
                    stop_line_ids_on_path.append(id)

        with self.lock:
            self.stop_line_ids_on_path = stop_line_ids_on_path
            self.transform_from_frame = local_path_msg.header.frame_id

    def camera_image_callback(self, camera_image_msg):

        if self.camera_model is None:
            rospy.logwarn_throttle(10, "%s - No camera model received, skipping image", rospy.get_name())
            return

        if self.stop_line_ids_on_path is None:
            rospy.logwarn_throttle(10, "%s - No path received, skipping image", rospy.get_name())
            return

        with self.lock:
            stop_line_ids_on_path = self.stop_line_ids_on_path
            transform_from_frame = self.transform_from_frame

        # camera image seems to be delayed compared to the position of the car, compensate for it
        image_time_stamp = camera_image_msg.header.stamp - rospy.Duration.from_sec(self.camera_delay_compensation)
        transform_to_frame = camera_image_msg.header.frame_id

        tfl_status = StopLineStatusArray()
        tfl_status.type = StopLineStatusArray.TRAFFIC_LIGHT
        tfl_status.header.stamp = image_time_stamp

        map_rois = []
        yolo_rois = []
        match_dict = {}

        # extract image
        image = self.bridge.imgmsg_to_cv2(camera_image_msg, desired_encoding='rgb8')

        # rectify image
        if self.rectify_image:
            self.camera_model.rectifyImage(image, image)

        if stop_line_ids_on_path:

            # Extract the transform between transform_to_frame and transform_from_frame
            try:
                transform = self.tf_buffer.lookup_transform(transform_to_frame,transform_from_frame,image_time_stamp)
            except (tf2_ros.TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return
            map_rois = self.calculate_roi_coordinates(stop_line_ids_on_path, transform)

            if map_rois:
                # Run the YOLO model on the image to get yolo_rois, classes and scores,
                # then match them with map_rois:
                yolo_rois, classes, scores = self.yolo_model.predict(image)
                # keep only the main traffic light classes, discard arrow-specific detections
                mask = classes < 4
                yolo_rois, classes, scores = yolo_rois[mask], classes[mask], scores[mask]
                tfl_results, match_dict = self.match_map_and_yolo_rois(map_rois,yolo_rois,classes,scores)
                tfl_status.statuses.extend(tfl_results)

        self.tfl_status_pub.publish(tfl_status)

        self.publish_roi_images(image, map_rois, yolo_rois, match_dict, image_time_stamp)

    def calculate_roi_coordinates(self, stop_line_ids_on_path, transform):
        rois = []

        for stop_line_id in stop_line_ids_on_path:
            for traffic_light_id, traffic_light_coords in self.traffic_lights[stop_line_id].items():
                us = []
                vs = []

                for x, y, z in traffic_light_coords:
                    point_map = Point(x=x, y=y, z=z)

                    # Find camera pixel coordinates 
                    point_camera = do_transform_point(PointStamped(point=point_map), transform).point
                    u, v = self.camera_model.project3dToPixel((point_camera.x, point_camera.y, point_camera.z))
                    if point_camera.z < 0:
                        break 
                    # convert the extent in meters to extent in pixels
                    extent_x_px = self.camera_model.fx() * self.roi_width_extent / point_camera.z
                    extent_y_px = self.camera_model.fy() * self.roi_height_extent / point_camera.z

                    us.extend([u + extent_x_px, u - extent_x_px])
                    vs.extend([v + extent_y_px, v - extent_y_px])

                # not all traffic light corners were in the image, take next traffic light
                if len(us) < 8:
                    continue

                # round and clip against image limits
                us = np.clip(np.round(np.array(us)), 0, self.camera_model.width - 1)
                vs = np.clip(np.round(np.array(vs)), 0, self.camera_model.height - 1)

                # extract one roi per traffic light
                min_u = int(np.min(us))
                max_u = int(np.max(us))
                min_v = int(np.min(vs))
                max_v = int(np.max(vs))

                # check if roi is too small
                if max_u - min_u < self.min_roi_width:
                    continue

                rois.append([stop_line_id, traffic_light_id, min_u, max_u, min_v, max_v])

        return rois

    def match_map_and_yolo_rois(self, map_rois, yolo_rois, yolo_classes, yolo_scores):
        tfl_results = []
        match_dict = {}

        # for every map roi
        for stop_line_id, traffic_light_id, x1_map, x2_map, y1_map, y2_map in map_rois:
            matched_roi = None
            highest_iou = self.iou_threshold
            # Find the YOLO detection that best matches this map ROI.
            for idx, (cls, score, yolo_roi) in enumerate(zip(yolo_classes, yolo_scores, yolo_rois)):
                iou_score = self.calculate_iou(np.array([[
                    x1_map, y1_map, x2_map, y2_map]]), yolo_roi[np.newaxis, :])[0][0]
                if iou_score < self.iou_threshold:
                    continue
                if matched_roi is None or iou_score > highest_iou:
                    matched_roi = [cls, score, yolo_roi, idx] 
                    highest_iou = iou_score
                    
            tfl_result = StopLineStatus()
            tfl_result.traffic_light_id = traffic_light_id
            tfl_result.stop_line_id = stop_line_id

            if matched_roi is None:
                # no match for map ROI - traffic light status is missing
                tfl_result.status = StopLineStatus.STATUS_MISSING
                tfl_result.status_text = "missing"
                match_dict[traffic_light_id] = None
            else:
                # yolo ROI and map ROI were matched
                match_dict[traffic_light_id] = matched_roi
                tfl_result.status = CLASS_TO_TLRESULT[matched_roi[0]]
                tfl_result.status_text = CLASS_TO_STRING[matched_roi[0]]
            
            tfl_results.append(tfl_result)
        return tfl_results, match_dict

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

    @staticmethod
    def calculate_iou(boxes1, boxes2):
        """
        Calculate the IOU between two sets of bounding boxes.
        :param boxes1: numpy array of shape (n, 4) with box coordinates in (x1, y1, x2, y2) format
        :param boxes2: numpy array of shape (m, 4) with box coordinates in (x1, y1, x2, y2) format
        :return: numpy array of shape (n, m) with the IOU between all pairs of boxes
        """
        # Calculate the area of each bounding box
        area1 = (boxes1[:, 2] - boxes1[:, 0]) * (boxes1[:, 3] - boxes1[:, 1])
        area2 = (boxes2[:, 2] - boxes2[:, 0]) * (boxes2[:, 3] - boxes2[:, 1])

        # Calculate the coordinates of the intersection bounding boxes
        intersection_x1 = np.maximum(boxes1[:, 0][:, np.newaxis], boxes2[:, 0])
        intersection_y1 = np.maximum(boxes1[:, 1][:, np.newaxis], boxes2[:, 1])
        intersection_x2 = np.minimum(boxes1[:, 2][:, np.newaxis], boxes2[:, 2])
        intersection_y2 = np.minimum(boxes1[:, 3][:, np.newaxis], boxes2[:, 3])

        # Calculate the area of the intersection bounding boxes
        intersection_area = np.maximum(intersection_x2 - intersection_x1, 0) * np.maximum(intersection_y2 - intersection_y1, 0)

        # Calculate the union of the bounding boxes
        union_area = area1[:, np.newaxis] + area2 - intersection_area

        return intersection_area / union_area

    def publish_roi_images(self, image, map_rois, yolo_rois, match_dict, image_time_stamp):
        # add rois to image
        if map_rois:
            matched_yolo_roi_idxs = []

            for _, traffic_light_id, min_u, max_u, min_v, max_v in map_rois:
                if match_dict.get(traffic_light_id) is None:
                    # map roi was not matched with any yolo roi
                    text_string = "%s %.2f" % ("missing", 0)
                    color = (200, 200, 200)

                else:
                    # map roi was matched with a yolo roi
                    cl, score, yolo_roi, yolo_idx = match_dict[traffic_light_id]

                    yolo_min_u, yolo_min_v, yolo_max_u, yolo_max_v = yolo_roi
                    matched_yolo_roi_idxs.append(yolo_idx)

                    # add smaller yolo roi
                    yolo_start_point = (yolo_min_u, yolo_min_v)
                    yolo_end_point = (yolo_max_u, yolo_max_v)
                    cv2.rectangle(image, yolo_start_point, yolo_end_point, color=CLASS_TO_LIGHT_COLOR[cl], thickness=2)

                    text_string = "%s %.2f" % (CLASS_TO_STRING[cl], score)
                    color = CLASS_TO_COLOR[cl]

                # add bigger map roi
                text_width, text_height = cv2.getTextSize(text_string, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 2)[0]
                text_orig_u = int(min_u + (max_u - min_u) / 2 - text_width / 2)
                text_orig_v = max_v + text_height + 3

                start_point = (min_u, min_v)
                end_point = (max_u, max_v)
                cv2.rectangle(image, start_point, end_point, color=color, thickness=3)
                cv2.putText(image,
                            text_string,
                            org=(text_orig_u, text_orig_v),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1.5,
                            color=color,
                            thickness=2)

            # add all yolo ROIs that were not matched to any map ROI
            for i, (yolo_min_u, yolo_min_v, yolo_max_u, yolo_max_v) in enumerate(yolo_rois):
                if i not in matched_yolo_roi_idxs:
                    yolo_start_point = (yolo_min_u, yolo_min_v)
                    yolo_end_point = (yolo_max_u, yolo_max_v)
                    cv2.rectangle(image, yolo_start_point, yolo_end_point, color=(0, 0, 255), thickness=2)

        image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding='rgb8')

        img_msg.header.stamp = image_time_stamp
        self.tfl_roi_pub.publish(img_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('yolo_traffic_light_detector', log_level=rospy.INFO)
    node = YoloTrafficLightDetector()
    node.run()
