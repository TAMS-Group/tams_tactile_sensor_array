import sys
import numpy as np
import rosbag
from matplotlib import pyplot as plt

force_times = list()
force_values = list()
data_times = list()
data_values = list()

with rosbag.Bag(sys.argv[1]) as bag:
    for topic, msg, t in bag.read_messages(topics=['/tactile_sensor_data/1', '/wireless_ft/wrench_3']):
        if topic == '/tactile_sensor_data/1':
            data_times.append(int(t.to_nsec()))
            data_values.append(int(1024 - msg.data[int(sys.argv[2])]))
        elif topic == '/wireless_ft/wrench_3':  # and msg.wrench.force.z > -2:
            force_times.append(int(t.to_nsec()))
            force_values.append(float(msg.wrench.force.z)*-10)
data_values_interp = np.interp(force_times, data_times, data_values)
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf', '#000000']
bins = np.arange(0.5, 10.5, .1)
bin_indices = np.digitize(force_values, bins)
plt.scatter(force_values, data_values_interp)
plt.boxplot([[data_values_interp[i] for i in range(len(force_values)) if bin_indices[i] == j] for j in range(1, len(bins), 10)])
plt.show()

