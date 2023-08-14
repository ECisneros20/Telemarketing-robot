import matplotlib.pyplot as plt
import rosbag
from rospkg import RosPack
import os

# rosbag record -O ./catkin_ws/src/Telemarketing-robot/telemarketing_microcontroller/tests/test1.bag /vel_setpoint /encoder_data

bag_name = input("Choose bag file: ")
rospack = RosPack()
package_path = rospack.get_path("telemarketing_microcontroller")
bag = rosbag.Bag(os.path.join(package_path, "tests", f"{bag_name}.bag"))
timestamps_setpoint = []
timestamps_encoder = []
values_setpoint_r = []
values_setpoint_l = []
values_encoder_r = []
values_encoder_l = []

for topic, msg, t in bag.read_messages(topics = ["/vel_setpoint"]):
    timestamps_setpoint.append(t.to_sec())
    values_setpoint_r.append(msg.data[0])
    values_setpoint_l.append(msg.data[1])

for topic, msg, t in bag.read_messages(topics = ["/encoder_data"]):
    timestamps_encoder.append(t.to_sec())
    values_encoder_r.append(msg.data[0])
    values_encoder_l.append(msg.data[1])

bag.close()

# Initialise the subplot function using number of rows and columns
figure, (ax1, ax2) = plt.subplots(2, 1, figsize = (15, 12))

# For the right motor
ax1.plot(timestamps_setpoint, values_setpoint_r, color = "g", label = "setpoint")
ax1.plot(timestamps_encoder, values_encoder_r, color = "r", label = "encoder")
ax1.set_xlabel("Time(s)")
ax1.set_ylabel("Angular velocity (rad/s)")
ax1.set_title("Right motor control response")
ax1.legend()

# For the left motor
ax2.plot(timestamps_setpoint, values_setpoint_l, color = "g", label = "setpoint")
ax2.plot(timestamps_encoder, values_encoder_l, color = "r", label = "encoder")
ax2.set_xlabel("Time(s)")
ax2.set_ylabel("Angular velocity (rad/s)")
ax2.set_title("Left motor control response")
ax2.legend()

# Combine all the operations and display
plt.show()
