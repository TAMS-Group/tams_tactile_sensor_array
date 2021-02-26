import sys
import numpy as np
import rosbag
import csv

# extracts the force data and sensor measurements of a single taxel out of a rosbag, interpolates the sensor values and
# writes all into a csv file
# call with rosbag filename as first argument and taxel id as second argument

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
            force_values.append(float(msg.wrench.force.z)*-1)
data_values_interp = np.interp(force_times, data_times, data_values)

with open(sys.argv[1].replace('.bag','') + '_taxel' + sys.argv[2] + '.csv', 'w') as f:
    csv_writer = csv.writer(f)
    csv_writer.writerow(["time", "force", "value"])
    for i in range(len(force_times)):
        csv_writer.writerow([force_times[i], force_values[i], data_values_interp[i]])
