#! /usr/bin/env python3
import serial
import bluetooth
from bitstring import ConstBitStream
import numpy as np
import struct
import rospy
import math
from tams_tactile_sensor_array.msg import TactileSensorArrayData

class SensorReader:
    def __init__(self):
        rospy.init_node('TactileSensorArray')


        self.use_bluetooth = rospy.get_param('/TactileSensor/tactile/use_bluetooth', False)

        if self.use_bluetooth:
            self.bt_sock = bluetooth.BluetoothSocket()
            self.bt_sock.connect((rospy.get_param('/TactileSensor/tactile/bluetooth_device_address'), 1))
        else:
            self.serial = serial.Serial(rospy.get_param('/TactileSensor/tactile/serial_port'), rospy.get_param('/TactileSensor/tactile/serial_baud'))

        self.sensors = dict()
        for sensor in rospy.get_param('/TactileSensor/tactile/sensors'):
            s = Sensor.from_dict(sensor)
            self.sensors[s.id] = s

        # wait until one transmission is completed.
        self.wait_until_end()

        while True:
            sensor_id, data = self.receive_one()
            if data: 
                self.sensors[sensor_id].publisher.publish(self.sensors[sensor_id].generate_data_msg(data))

    def receive_one(self):
        if self.use_bluetooth:
            d = self.bt_sock.recv(1)
        else:
            d = self.serial.read(1)
        sensor_number = struct.unpack('B', d)[0]
        return sensor_number, self.receive_sensor(sensor_number)

    def receive_sensor(self, sensor_number):
        if sensor_number not in self.sensors.keys():
            print("Skipping sensor_number \"" + sensor_number + "\"")
            self.wait_until_end()
            return None
        sensor = self.sensors[sensor_number]  # type: Sensor
        if self.use_bluetooth:
            raw_data = self.bt_sock.recv(sensor.request_size)
            while len(raw_data) < sensor.request_size:
                raw_data += self.bt_sock.recv(sensor.request_size - len(raw_data))
        else:
            raw_data = self.serial.read(sensor.request_size)

        data = []
        stream = ConstBitStream(raw_data)
        for x in range(sensor.width * sensor.height):
            data.append(stream.read('uint:10'))
        return data

    def wait_until_end(self):
        # wait until one transmission is completed.
        checkbyte = 42
        while checkbyte != bytes.fromhex('00'):
            if self.use_bluetooth:
                checkbyte = self.bt_sock.recv(1)
            else:
                checkbyte = self.serial.read(1)
            # print(checkbyte)


class Sensor:

    def __init__(self, sensor_id, width, height, value_size, frame, active=True):
        self.id = sensor_id
        self.width = width
        self.height = height
        self.value_size = value_size
        self.frame = frame
        self.request_size = int(math.ceil(5/4 * self.width * self.height) + 1)
        self. active = active
        self.publisher = rospy.Publisher(rospy.get_param('/TactileSensor/tactile/sensor_data_namespace') + '/' + str(sensor_id), TactileSensorArrayData, queue_size=1)

    def generate_data_msg(self, data):
        msg = TactileSensorArrayData()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.data = data
        msg.sensor_data_height = self.height
        msg.sensor_data_width = self.width
        msg.sensor_id = self.id
        # print(msg)
        return msg

    @staticmethod
    def from_dict(sensor_dict):
        return Sensor(
            sensor_dict['id'],
            sensor_dict['width'],
            sensor_dict['height'],
            sensor_dict['value_size'],
            sensor_dict['frame'],
            sensor_dict.get('active', True)
        )


if __name__ == '__main__':
    SensorReader()
