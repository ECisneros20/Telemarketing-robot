# Telemarketing-robot

## Description

 This repository contains the virtual model of a telemarketing robot to evaluate autonomous navigation algorithms, as well as what is needed to perform tests on a physical robot. This project was developed and explained in the papers [1] [2].

## Installation

Open a new terminal and type the following path:

    cd ~/catkin_ws/src

To clone the latest stable version into your workspace:

    git clone https://github.com/ECisneros20/Telemarketing-robot.git

To clone the latest version into your workspace:

    git clone -b release/1.0.0 https://github.com/ECisneros20/Telemarketing-robot.git

## Usage for physical tests

For the local computer (sentiment analysis + microcontroller and computer interaction via python and ino scripts):

    roslaunch telemarketing_bringup telemarketing_bringup_local.launch

For the remote computer (RViz, URDF and Gazebo configuration + gui interaction + teleoperation node):

    roslaunch telemarketing_bringup telemarketing_bringup_remote.launch

## Next steps

- Organize according to the ros naming standard.

- Create new packages and ros launch scripts.

- According to the new changes, complete the Usage section.

## License

MIT License

## References

[1] D. Arce <em>et al.</em>, ”Design and Implementation of Telemarketing Robot with Emotion Identification for Human-Robot Interaction,” <em>2022 Sixth IEEE International Conference on Robotic Computing (IRC)</em>, Italy, 2022, pp. 177-180, doi: 10.1109/IRC55401.2022.00037.

[2]

[3] <a href = "https://automaticaddison.com/naming-and-organizing-packages-in-large-ros-2-projects/">Naming and Organizing Packages in Large ROS 2 Projects</a>
