^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for Telemarketing robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.3.1 (2023-04-24)
------------------
* Updates in package organization and requirements.
* New packages, launch files and python scripts. Especially the bringup package.
* (ASP) TODO: Define the python libraries version in the requirements.txt file
* (ASP) TODO: Usage of ROS in the telemarketing_gui package in order to check camera image, battery level, velocity, etc
* (ASP) TODO: Successful teleoperation via ps4 controller connected to a remote PC with ROS in the telemarketing_microcontroller package
    - The subscriber to /cmd_vel topic
    - The controller algorithm and fine-tunning for driver commands
    - Feedback from the encoder measurements
* TODO: Fusion of all sensor data in the same script with the proper subscribers and publishers in telemarketing_microcontroller package
* TODO: Define of the algorithm to stimate Odometry message based only in the encoders data and TF messages in telemarketing_microcontroller package
* TODO: Include ROS characteristics to sentiment_analysis package

0.3.0 (2023-04-24)
------------------
* Updates in package organization and requirements.
* New launch files and python scripts.

0.2.0 (2023-04-15)
------------------
* Updates in URDF file, teleop files and arduino and python scripts.

0.1.0 (2023-02-11)
------------------
* First release 0.1.0 with several changes.

0.0.5 (2023-01-24)
------------------
* Changes in telemarketing_microcontroller.

0.0.4 (2023-01-23)
------------------
* Changes in telemarketing_microcontroller.

0.0.3 (2023-01-22)
------------------
* Update README.md file and some packages.

0.0.2 (2022-07-18)
------------------
* Update sentiment_analysis (complete) and telemarketing_teleop packages (src to do).

0.0.1 (2022-07-18)
------------------
* Initial development of sentiment_analysis, telemarketing_description, telemarketing_gazebo and telemarketing_teleop packages.
