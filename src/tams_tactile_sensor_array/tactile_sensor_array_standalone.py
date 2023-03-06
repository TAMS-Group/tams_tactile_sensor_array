#! /usr/bin/env python3
import serial
import bluetooth
from bitstring import ConstBitStream
import numpy as np
import struct
import math
import yaml
import time

class SensorReader:
    def __init__(self, config_dict, continuous_run=False, callback_function=None):
        self.config = config_dict
        self.use_bluetooth = self.config['use_bluetooth']
        self.callback = callback_function

        if self.use_bluetooth:
            self.bt_sock = bluetooth.BluetoothSocket()
            self.bt_sock.connect((self.config['bluetooth_device_address'], 1))
        else:
            self.serial = serial.Serial(self.config['serial_port'], self.config['serial_baud'])

        self.sensors = dict()
        for sensor in self.config['sensors']:
            s = Sensor.from_dict(sensor)
            self.sensors[s.id] = s

        # wait until one transmission is completed.
        self.wait_until_end()
        if continuous_run:
            if not self.callback:
                raise AttributeError('Callback function missing!')
            while True:
                sensor_id, data = self.receive_one()
                if data:
                    self.callback(data)

    def receive_one(self):
        # make sure that a previous transmission was completed.
        if self.use_bluetooth:
            d = self.bt_sock.recv(1)
        else:
            d = self.serial.read(1)
        sensor_number = struct.unpack('B', d)[0]
        return sensor_number, self.receive_sensor(sensor_number)

    def receive_sensor(self, sensor_number):
        if sensor_number not in self.sensors.keys():
            print("Skipping unknown sensor with sensor_number \"" + str(sensor_number) + "\"")
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

    def generate_data_dict(self, data):
        return {
            'timestamp': time.time(),
            'data': data,
            'sensor_data_height': self.height,
            'sensor_data_width': self.width,
            'sensor_id': self.id,
        }

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
    with open('../../config/params_diana.yaml', 'r') as f:
        config = yaml.safe_load(f)

    SensorReader(config['tactile'], continuous_run=True, callback_function=print)
