#!/usr/bin/env python3

import rospy
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured, unstructured_to_structured
from sklearn.cluster import DBSCAN

from ros_numpy import numpify, msgify
from sensor_msgs.msg import PointCloud2


class PointsClusterer:
    def __init__(self):
        # Parameters
        self.cluster_epsilon = rospy.get_param('~cluster_epsilon')
        self.cluster_min_samples = rospy.get_param('~cluster_min_samples')

        # Create self.clusterer using DBSCAN with the parameters above.
        self.clusterer = DBSCAN(eps=self.cluster_epsilon, min_samples=self.cluster_min_samples)

        # Publishers
        self.clustered_pub = rospy.Publisher('points_clustered', PointCloud2, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('points_filtered', PointCloud2, self.points_callback, queue_size=1, buff_size=2**24, tcp_nodelay=True)

        rospy.loginfo("%s - initialized", rospy.get_name())

    def points_callback(self, msg):

        # Extract points from the message and cluster them.
        data = numpify(msg)
        points = structured_to_unstructured(data[['x', 'y', 'z']], dtype=np.float32)
        labels = self.clusterer.fit_predict(points)
        if np.size(points) == 0:
            return 

        # Publish the clustered points as a PointCloud2 message.
        # Concatenate points with labels
        points_labeled = np.concatenate((points,labels[:,None]),axis=1)

        # Filter out noise points (label == -1)
        points_labeled = points_labeled[points_labeled[:,3]!=-1]

        # Convert to structured PointCloud2 format
        data = unstructured_to_structured(points_labeled, dtype=np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('label', np.int32)
        ]))

        # Create the message using msgify, set the correct header and publish
        msg_to_publish = msgify(PointCloud2,data)
        msg_to_publish.header.stamp = msg.header.stamp
        msg_to_publish.header.frame_id = msg.header.frame_id
        self.clustered_pub.publish(msg_to_publish)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('points_clusterer', log_level=rospy.INFO)
    node = PointsClusterer()
    node.run()