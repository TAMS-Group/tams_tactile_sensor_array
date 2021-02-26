import sys
import numpy as np
import rosbag

# prints the taxel id of the most used taxel
# call with rosbag filename as first argument

taxel_count = 36
taxel_sum = np.zeros([taxel_count])
bias = np.ones([taxel_count]) * 1024

with rosbag.Bag(sys.argv[1]) as bag:
    for topic, msg, t in bag.read_messages(topics=['/tactile_sensor_data/1', '/wireless_ft/wrench_3']):
        if topic == '/tactile_sensor_data/1':
            taxel_sum += bias - np.array(msg.data)

print(np.argmax(taxel_sum))
