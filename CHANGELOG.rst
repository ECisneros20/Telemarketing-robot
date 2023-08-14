^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for Telemarketing robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.3.9 (2023-08-14)
------------------
* Improve the README file documentation.
* Several updates of sentiment_analysis, telemarketing_bringup, telemarketing_description, telemarketing_gazebo, telemarketing_microcontroller,
    telemarketing_teleop packages.
* Include ROS characteristics to sentiment_analysis package.
* Update in the telemarketing_bringup launch files.
* Basic teleoperation via ps4 controller connected to a remote PC with ROS in the telemarketing_microcontroller package.
    - Python script (teleop) with the subscriber to /cmd_vel and the publisher /servo_vel to the .ino file
    - .ino file with the subscriber to /servo_vel in order to use the servo commands for each motor
* (ASP) TODO: First upgrade in teleoperation, second file in charge of the controller.
    - Python script (controller) with the subscriber to /encoder_data and /servo_vel, and the publisher /servo_vel_controlled to the .ino file
    - .ino file with the subscriber to /servo_vel_controlled and include the two encoder counters for the publisher /encoder_data
    - The controller algorithm and fine-tunning for driver commands
    - Check Controller.py, ROBOTEQ.py and Serial.py
* (ASP) TODO: Use of ROS in the telemarketing_gui package in two scripts, one for each gui. The operator's gui is to check the 3D camera image, battery
    level, speed and detected sentiment. The customer's gui is for displaying advertisements and other related information. The development will be based
    in the virtual_joystick.py file: https://github.com/jfstepha/differential-drive/blob/master/scripts/virtual_joystick.py.
* (ASP) TODO: Develop python script to get the host pc name with its IP and automatically send via email as soon as the Jetson computers and Raspberry are
    turned on.
* TODO: Create script (sensors) to get the ultrasonic, infrared and bumper information with the subscriber to /sensor_data and the publisher for each sensor.
* TODO: Create script (localization) to define of the algorithm to estimate Odometry message based only in the encoders data and TF messages in
    telemarketing_microcontroller package.
* TODO: Fusion of all sensor data in the same script (localization) with the proper subscribers and publishers in telemarketing_microcontroller package.
    - Lidar
    - Encoders
    - IMU
* TODO: Use tensorRT in order to simplify the sentiment analysis model and speed up while executing it.

0.3.8 (2023-07-27)
------------------
* Create new branch feature/GUI for the telemarketing_gui package development.

0.3.7 (2023-07-27)
------------------
* Update of the telemarketing_gui package.

0.3.6 (2023-07-27)
------------------
* Add extra files and folders.

0.3.5 (2023-06-30)
------------------
* Update of the telemarketing_description and telemarketing_microcontroller packages.

0.3.4 (2023-05-13)
------------------
* Update of the telemarketing_description package and some other scripts.

0.3.3 (2023-05-01)
------------------
* New approach for odometry, pid control and tf calculations.

0.3.2 (2023-04-25)
------------------
* Updates in python scripts.

0.3.1 (2023-04-24)
------------------
* Updates in package organization and requirements.
* New packages, launch files and python scripts. Especially the bringup package.

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
