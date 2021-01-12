# A message containing a tactile sensor array measurement

std_msgs/Header header

# id of the sensor
uint8 sensor_id

# Number of taxels along the x-axis of the sensor
uint8 sensor_data_width
# Number of taxels along the y-axis of the sensor
uint8 sensor_data_height

# Sensor data
int32[] data