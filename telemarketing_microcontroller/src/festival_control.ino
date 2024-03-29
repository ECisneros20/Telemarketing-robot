#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>
// Link: https://www.researchgate.net/figure/Encoder-Channel-Description-The-encoder-read-values-for-channel-A-and-channel-B-can-be_fig3_329479537

// Encoders setup
volatile long int ticksR = 0;
volatile long int ticksL = 0;
unsigned long TimeBackup = 0;
const int encoderR[2] = {3, 2}; // {channelA, channelB}
const int encoderL[2] = {19, 18}; // {channelA, channelB}
const int TICKS_PER_REVOLUTION = 8000;
// Reset bumper
const int reset = 48;
// Motors
Servo MotorR;
Servo MotorL;

// Callback declaration
void messageCb(const std_msgs::Int32MultiArray &msg);

// ROS node declaration
ros::NodeHandle nh;
// ROS subscriber
ros::Subscriber<std_msgs::Int32MultiArray> sub("/servo_vel_controlled", &messageCb);
// ROS publisher
std_msgs::Float32MultiArray msg;
ros::Publisher pub("/encoder_data", &msg);

void setup() {

    Serial.begin(57600);
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
    nh.advertise(pub);
    pinMode(reset, OUTPUT);
    MotorR.attach(5);
    MotorL.attach(6);
    attachInterrupt(digitalPinToInterrupt(encoderR[0]), encoderRcallback, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoderL[0]), encoderLcallback, FALLING);
    setMotorR(1500);
    setMotorL(1500);
    // Bumper initialization
    digitalWrite(reset, LOW);
    //digitalWrite(reset, HIGH);

}

void loop() {

    if (millis() - TimeBackup >= 100) { // 100 ms

        float wR = ticksR / (0.1 * TICKS_PER_REVOLUTION) * 2 * PI;
        float wL = ticksL / (0.1 * TICKS_PER_REVOLUTION) * 2 * PI;
        float w[2] = {wR, wL};
        msg.data_length = 2;
        msg.data = w;
        pub.publish(&msg);
        nh.spinOnce();
        TimeBackup = millis();
        ticksR = 0;
        ticksL = 0;
        nh.spinOnce();

    }

}

void setMotorR(int pulse) {
    MotorR.writeMicroseconds(pulse);
}

void setMotorL(int pulse) {
    MotorL.writeMicroseconds(pulse);
}

// Callback definition
void messageCb(const std_msgs::Int32MultiArray &msg) {
    setMotorR(msg.data[0]);
    setMotorL(msg.data[1]);
}

void encoderRcallback() {
    if (digitalRead(encoderR[0]) == digitalRead(encoderR[1])) {
        ticksR++;
    }
    else {
        ticksR--;
    }
}

// Assuming a counter-intuitive reasoning
void encoderLcallback() {
    if (digitalRead(encoderL[0]) != digitalRead(encoderL[1])) {
        ticksL--;
    }
    else {
        ticksL++;
    }
}
