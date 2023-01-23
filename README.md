# Telemarketing-robot

## Description

 This repository contains the virtual model of a telemarketing robot to evaluate autonomous navigation algorithms, as well as what is needed to perform tests on a physical robot. This project was developed and explained in the papers [1] [2].

## Installation

Open a new terminal and type the following path:

    cd ~/catkin_ws/src

To clone the latest stable version into your workspace:

    git clone https://github.com/ECisneros20/Telemarketing-robot.git

To clone the latest version into your workspace:

    git clone -b testing https://github.com/ECisneros20/Telemarketing-robot.git

## Usage for virtual tests

Spawn the robot in the Gazebo environment:

    roslaunch telemarketing_description spawn_robot.launch

## Usage for physical tests

Spawn the robot in the Gazebo environment:

    roslaunch telemarketing_description spawn_robot.launch

## Next steps

### Before new packages

    - Translate every script.

    - Organize according to the ros naming standard [3].

    - Create new packages and ros launch scripts.

### After new packages

    - Translate every script.

    - Organize according to the ros naming standard.

    - Create new packages and ros launch scripts.

    - According to the new changes, complete the Usage section.

## License

MIT License

## References

[1]

[2]

[3] <a href = "https://automaticaddison.com/naming-and-organizing-packages-in-large-ros-2-projects/">Naming and Organizing Packages in Large ROS 2 Projects</a>
