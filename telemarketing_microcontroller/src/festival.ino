#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>

// Reset bumper
const int reset = 48;
// Motors
Servo MotorR;
Servo MotorL;

// Callback declaration
void messageCb(const std_msgs::Int32MultiArray& msg);

// ROS node declaration
ros::NodeHandle nh;
// ROS subscriber
ros::Subscriber<std_msgs::Int32MultiArray> sub("/servo_vel", &messageCb);

void setup() {

    Serial.begin(57600);
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
    pinMode(reset, OUTPUT);
    MotorR.attach(5);
    MotorL.attach(6);
    setMotorR(1500);
    setMotorL(1500);
    // Bumper initialization
    digitalWrite(reset, LOW);
    //digitalWrite(reset,HIGH);

}

void loop() {

    nh.spinOnce();

}

void setMotorR(int pulse) {
    MotorR.writeMicroseconds(pulse);
}

void setMotorL(int pulse) {
    MotorL.writeMicroseconds(pulse);
}

// Callback definition
void messageCb(const std_msgs::Int32MultiArray& msg) {
    setMotorR(msg.data[0]);
    setMotorL(msg.data[1]);
}
