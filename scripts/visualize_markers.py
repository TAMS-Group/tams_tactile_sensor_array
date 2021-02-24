#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tams_tactile_sensor_array.msg import TactileSensorArrayData

class TactileMarker:
    def __init__(self):
        self.pub = rospy.Publisher('robotiq/tactile_data', MarkerArray, queue_size=1)

        self.sub_finger_1 = rospy.Subscriber(rospy.get_param('/TactileSensor/tactile/sensor_data_namespace') + '/' + '2', TactileSensorArrayData, self.callback_1)
        self.sub_finger_2 = rospy.Subscriber(rospy.get_param('/TactileSensor/tactile/sensor_data_namespace') + '/' + '3', TactileSensorArrayData, self.callback_2)
        self.sub_middle = rospy.Subscriber(rospy.get_param('/TactileSensor/tactile/sensor_data_namespace') + '/' + '1', TactileSensorArrayData, self.callback_middle)

        self.init_finger_1 = False
        self.init_finger_2 = False
        self.init_middle = False

        self.marker_msg = MarkerArray()

        self.marker_msg.markers.append(Marker())
        self.marker_msg.markers.append(Marker())
        self.marker_msg.markers.append(Marker())

        # colors:
        # from 0 to thr_1, from blue to green
        # from thr_1 to thr_2, from green to yellow
        # from thr_2 to thr_3, from yellow to red
        self.thr_1 = 200
        self.thr_2 = 600
        self.thr_3 = 800


    def callback_1(self, msg):
        if not self.init_finger_1:
            self.marker_msg.markers[0] = self.cube_list(msg.sensor_data_width, msg.sensor_data_height)
            self.marker_msg.markers[0].id = 0
            self.init_finger_1 = True
        self.marker_msg.markers[0].header = msg.header

        for i in range(len(msg.data)):
            self.get_color(self.marker_msg.markers[0].colors[i], msg.data[i])

        self.pub.publish(self.marker_msg)

    def callback_2(self, msg):
        if not self.init_finger_2:
            self.marker_msg.markers[1] = self.cube_list(msg.sensor_data_width, msg.sensor_data_height)
            self.marker_msg.markers[1].id = 1
            self.init_finger_2 = True
        self.marker_msg.markers[1].header = msg.header

        for i in range(len(msg.data)):
            self.get_color(self.marker_msg.markers[1].colors[i], msg.data[i])
        #self.pub.publish(self.marker_msg)

    def callback_middle(self, msg):
        if not self.init_middle:
            self.marker_msg.markers[2] = self.cube_list(msg.sensor_data_width, msg.sensor_data_height)
            self.marker_msg.markers[2].id = 2
            self.init_middle = True
        self.marker_msg.markers[2].header = msg.header

        for i in range(len(msg.data)):
            self.get_color(self.marker_msg.markers[2].colors[i], msg.data[i])
        #self.pub.publish(self.marker_msg)

    def get_color(self, color, value):
        value = float(value)
        if value < self.thr_1:
            color.r = 0.0
            color.g = 1.0 # value / self.thr_1
            color.b = 1.0 - (value / self.thr_1)
        elif value < self.thr_2:
            color.r = (value - self.thr_1) / (self.thr_2 - self.thr_1)
            color.g = 1.0
            color.b = 0.0
        elif value < self.thr_3:
            color.r = 1.0
            color.g = 1.0 - ((value - self.thr_2) / (self.thr_3 - self.thr_2))
            color.b = 0.0
        else:
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0

    def cube_list(self, height, width):
        marker = Marker()

        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.position.x = 0.01
        marker.pose.position.y = 0.015
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.259
        marker.pose.orientation.w = 0.966
        marker.scale.x = 0.003
        marker.scale.y = 0.003
        marker.scale.z = 0.003

        for x in range(height):
            for z in range(width):
                point = Point()
                point.x = height - x * 0.0038
                point.y = 0
                # - width/2 - taxcel_width/2 * scale
                point.z = (z - 2.5) * 0.0038
                marker.points.append(point)
                color = ColorRGBA()
                color.r = 1
                color.g = 1
                color.b = 1
                color.a = 1
                marker.colors.append(color)

        return marker

if __name__ == '__main__':
    rospy.init_node('roboiq_tactile_marker', anonymous=True)
    tm = TactileMarker()
    rospy.spin()
