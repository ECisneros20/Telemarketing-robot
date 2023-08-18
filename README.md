# Telemarketing-robot

## Description

This repository contains the virtual model of a telemarketing robot to evaluate autonomous navigation algorithms, as well as what is needed to perform tests on a physical robot. This project was developed and explained in the papers [1] [2].

## Installation

Open a new terminal and type the following path:

    cd ~/catkin_ws/src

To clone the most stable release into your workspace:

    git clone https://github.com/ECisneros20/Telemarketing-robot.git

To clone the latest release into your workspace:

    git clone -b release/1.0.0 https://github.com/ECisneros20/Telemarketing-robot.git

## Usage for physical tests

### For the Jetson Xavier NX local computer (sentiment analysis + clients gui interaction):

<table align="center">
  <tr>
    <th>Arg name</th>
    <th>Default value</th>
    <th>Package</th>
    <th>Other values</th>
  </tr>
  <tr>
    <td>show_sentiment_analysis</td>
    <td>False</td>
    <td>telemarketing_bringup</td>
    <td>True</td>
  </tr>
  <tr>
    <td>show_gui_customers</td>
    <td>False</td>
    <td>telemarketing_bringup</td>
    <td>True</td>
  </tr>
</table>

    roslaunch telemarketing_bringup telemarketing_bringup_local_xavier.launch

### For the Jetson Nano local computer (microcontroller and computer interaction via python and ino scripts):

<table align="center">
  <tr>
    <th>Arg name</th>
    <th>Default value</th>
    <th>Package</th>
    <th>Other values</th>
  </tr>
  <tr>
    <td>use_sensors</td>
    <td>none</td>
    <td>telemarketing_bringup</td>
    <td>only_lidar, only_camera and both</td>
  </tr>
  <tr>
    <td>use_arduino</td>
    <td>True</td>
    <td>telemarketing_microcontroller</td>
    <td>False</td>
  </tr>
</table>

    roslaunch telemarketing_bringup telemarketing_bringup_local_nano.launch

### For the remote computer (RViz, URDF and Gazebo configuration + operator gui interaction + teleoperation node):

  <arg name="rvizconfig" default="urdf_ir_us"/>
  <arg name="show_gazebo" default="False"/>
  <arg name="show_rviz" default="False"/>
  <arg name="show_gui_operator" default="False"/>
  <arg name="joy_config" default="ps4"/>


<table align="center">
  <tr>
    <th>Arg name</th>
    <th>Default value</th>
    <th>Package</th>
    <th>Other values</th>
  </tr>
  <tr>
    <td>rvizconfig</td>
    <td>urdf_ir_us</td>
    <td>telemarketing_description</td>
    <td>urdf, urdf_all_sensors</td>
  </tr>
  <tr>
    <td>show_gazebo</td>
    <td>False</td>
    <td>telemarketing_description</td>
    <td>True</td>
  </tr>
  <tr>
    <td>show_rviz</td>
    <td>False</td>
    <td>telemarketing_description</td>
    <td>True</td>
  </tr>
  <tr>
    <td>show_gui_operator</td>
    <td>False</td>
    <td>telemarketing_bringup</td>
    <td>True</td>
  </tr>
  <tr>
    <td>joy_config</td>
    <td>ps4</td>
    <td>telemarketing_teleop</td>
    <td>logitech, ps4-nocontrol</td>
  </tr>
</table>

    roslaunch telemarketing_bringup telemarketing_bringup_remote.launch

### Motor controller fine-tunning

Use the following command

    rosbag record -O ./catkin_ws/src/Telemarketing-robot/telemarketing_microcontroller/tests/test1.bag /vel_setpoint /encoder_data

Analyze the response for each motor

<br/>
<p align="center">
  <img src="https://user-images.githubusercontent.com/88266673/260391956-c1d8686e-5757-42ea-9be7-48fef25caff0.png" width="1000">
</p>

### Software changes for customers' GUI

List the input options

    xinput -list

Change the input for the multi touch

    xinput set-prop "Multi touch   Multi touch overlay device" --type=float "Coordinate Transformation Matrix" 0 -1 1 1 0 0 0 0 1

## Next steps

- Create new packages and roslaunch scripts.

- According to the new changes, complete the Usage section.

## License

MIT License

## References

[1] D. Arce <em>et al.</em>, ”Design and Implementation of Telemarketing Robot with Emotion Identification for Human-Robot Interaction,” <em>2022 Sixth IEEE International Conference on Robotic Computing (IRC)</em>, Italy, 2022, pp. 177-180, doi: 10.1109/IRC55401.2022.00037.

[2] 

[3] <a href = "https://automaticaddison.com/naming-and-organizing-packages-in-large-ros-2-projects/">Naming and Organizing Packages in Large ROS 2 Projects</a>
