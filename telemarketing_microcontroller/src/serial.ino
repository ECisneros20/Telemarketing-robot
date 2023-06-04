#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>

// Infrared sensors
const int IR[8] = {0, 1, 2, 3, 4, 5, 6, 7};
// Ultrasonic sensors
const int US[8] = {8, 9, 10, 11, 12, 13, 14, 15};
// Encoder ticks
volatile int ticksR = 0;
volatile int ticksL = 0;
const int encoderR[2] = {3, 2}; // {channelA, channelB}
const int encoderL[2] = {19, 18}; // {channelA, channelB}
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
volatile unsigned long TimeBackup = 0;
// Array of 16 sensors + 2 encoder signals + bumpers
float sensor_data[19];
// Motors
Servo MotorR;
Servo MotorL;

// ROS node declaration
ros::NodeHandle nh;
// ROS subscriber
ros::Subscriber<std_msgs::Int32MultiArray> sub("/servo_vel", &messageCb);
// ROS publisher
std_msgs::Float32MultiArray msg;
ros::Publisher pub("/sensor_data", &msg);

void setup() {

    Serial.begin(57600);
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.advertise(pub);
    nh.subscribe(sub);

    for (int i=0; i<=1; i++) {
        pinMode(encoderR[i], INPUT);
        pinMode(encoderL[i], INPUT);
    }

    MotorR.attach(5);
    MotorL.attach(6);
    attachInterrupt(digitalPinToInterrupt(encoderL[0]), encoderLcallback, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoderR[0]), encoderRcallback, FALLING);

}

void loop() {

    for (int i=0; i<=7; i++) {
        sensor_data[i] = read_US(i);
    }

    for (int i=8; i<=15; i++) {
        sensor_data[i] = read_IR(i);
    }

    if (millis()-TimeBackup>=100) { //100 ms

        sensor_data[16] = ticksR;
        sensor_data[17] = ticksL;

        // Set the message data
        msg.data_length = 18;
        msg.data = sensor_data;

        // Publish the message
        pub.publish(&msg);
        nh.spinOnce();
        TimeBackup = millis();
    }

}

float read_IR(int sensor) {

    float raw = 0, distance = 0;

    for (int i=0; i<10; i++) {
        raw = analogRead(IR[sensor]);
        distance += 13*pow(raw*0.0048828125,-1);
        delay(10);
    }

    return distance/1000;

}

float read_US(int sensor) {

    float raw = 0, distance = 0;

    for (int i=0; i<10; i++) {
        raw = analogRead(US[sensor]);
        distance += raw*0.5;
        delay(10);
    }

    return distance/1000;
}

void encoderLcallback() {

    if (digitalRead(encoderL[0]) != digitalRead(encoderL[1])) {
        if (ticksL == encoder_maximum)
            ticksL = encoder_minimum;
        else
            ticksL++;
    }
    else {
        if (ticksL == encoder_minimum)
            ticksL = encoder_maximum;
        else
            ticksL--;
    }

}

void encoderRcallback() {

    if (digitalRead(encoderR[0]) == digitalRead(encoderR[1])) {
        if (ticksR == encoder_maximum)
            ticksR = encoder_minimum;
        else
            ticksR++;
    }
    else {
        if (ticksR == encoder_minimum)
            ticksR = encoder_maximum;
        else
            ticksR--;
    }

}

void setMotorR(int pulse){

    MotorR.writeMicroseconds(pulse);

}

void setMotorL(int pulse){

    MotorL.writeMicroseconds(pulse);

}

void messageCb(const std_msgs::Int32MultiArray& msg) {

    setMotorR(msg[0]);
    setMotorL(msg[1]);

}
