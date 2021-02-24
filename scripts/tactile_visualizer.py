#! /usr/bin/env python
import numpy as np

import rospy
from tams_tactile_sensor_array.msg import TactileSensorArrayData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SensorDataVisualizer:
    def __init__(self, sensor_id):

        self.data_subscriber = rospy.Subscriber(
            rospy.get_param('/TactileSensor/tactile/sensor_data_namespace') + '/' + str(sensor_id),
            TactileSensorArrayData,
            self.receive_data
        )
        self.base_topic = 'tactile_viz_'
        self.data_scale = 60

        self.publishers = dict()
        self.cv_bridge = CvBridge()




    def receive_data(self, msg):
        # print(msg)
        #get publisher:
        publisher = self.publishers.get(msg.sensor_id)
        if not publisher:  # a new publisher has to be instantiated
            publisher = rospy.Publisher(self.base_topic + str(msg.sensor_id), Image, queue_size=1)
            self.publishers[msg.sensor_id] = publisher
        data = np.array(msg.data).reshape((msg.sensor_data_width, msg.sensor_data_height)) / 4
        data = data.astype(np.uint8)
        img = cv2.resize(
            data,
            (msg.sensor_data_width * self.data_scale, msg.sensor_data_height * self.data_scale),
            interpolation=cv2.INTER_CUBIC
        )
        img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
        cv2.imshow(self.base_topic + str(msg.sensor_id), img)
        cv2.waitKey(1)
        return
        img_msg = self.cv_bridge.cv2_to_imgmsg(img, "bgr8")
        img_msg.header = msg.header
        publisher.publish(img_msg)



if __name__ == '__main__':
    rospy.init_node('TactileSensorVisualizer')
    s1 = SensorDataVisualizer(1)

    rospy.spin()