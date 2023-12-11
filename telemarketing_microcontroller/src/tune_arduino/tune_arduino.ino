#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <Servo.h>
// Link: https://www.researchgate.net/figure/Encoder-Channel-Description-The-encoder-read-values-for-channel-A-and-channel-B-can-be_fig3_329479537

// Encoders setup
volatile long int ticksR = 0;
volatile long int ticksL = 0;
unsigned long TimeBackup = 0;
const int encoderR[2] = {3, 2}; // {channelA, channelB}
const int encoderL[2] = {19, 18}; // {channelA, channelB}
const int TICKS_PER_REVOLUTION = 1600;
// Reset bumper
const int reset = 48;
// Motors
Servo MotorR;
Servo MotorL;

// Callback declaration
void messageLeftCb(const std_msgs::Float64 &msg);
void messageRightCb(const std_msgs::Float64 &msg);

// ROS node declaration
ros::NodeHandle nh;
// ROS subscriber
ros::Subscriber<std_msgs::Float64> sub_left("/servo_vel_controlled_left", &messageLeftCb);
ros::Subscriber<std_msgs::Float64> sub_right("/servo_vel_controlled_right", &messageRightCb);
// ROS publisher
std_msgs::Float64 encoder_left;
std_msgs::Float64 encoder_right;
ros::Publisher pub_left("/encoder_data_left", &encoder_left);
ros::Publisher pub_right("/encoder_data_right", &encoder_right);

void setup() {

    Serial.begin(57600);
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub_left);
    nh.subscribe(sub_right);
    nh.advertise(pub_left);
    nh.advertise(pub_right);
    pinMode(reset, OUTPUT);
    MotorR.attach(5);
    MotorL.attach(6);
    pinMode(encoderR[0],INPUT_PULLUP);
    pinMode(encoderR[1],INPUT_PULLUP);
    pinMode(encoderL[0],INPUT_PULLUP);
    pinMode(encoderL[1],INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderR[0]), encoderRcallback, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoderL[0]), encoderLcallback, FALLING);
    setMotorR(1500);
    setMotorL(1500);
    // Bumper initialization
    digitalWrite(reset, LOW);
    //digitalWrite(reset, HIGH);

}

void loop() {

    if (millis() - TimeBackup >= 10) { // 10 ms

        float wR = ticksR / (0.01 * TICKS_PER_REVOLUTION) * 2 * PI;
        float wL = ticksL / (0.01 * TICKS_PER_REVOLUTION) * 2 * PI;
        //float w[2] = {wR, wL};
        encoder_left.data = wL;
        encoder_right.data = wR;
        pub_left.publish(&encoder_left);
        pub_right.publish(&encoder_right);
        //nh.spinOnce();
        TimeBackup = millis();
        ticksR = 0;
        ticksL = 0;
        //nh.spinOnce();

    }
        nh.spinOnce();
}

void setMotorR(int pulse) {
  if (pulse>1800 || pulse<1200)
    pulse=1500;
  MotorR.writeMicroseconds(pulse);
}

void setMotorL(int pulse) {
  if (pulse>1800 || pulse<1200)
    pulse=1500;
  MotorL.writeMicroseconds(pulse);
}

// Callback definition
void messageLeftCb(const std_msgs::Float64 &msg) {
    int left_pulse=1200 + (msg.data-(-10))*1.0*(1800-1200)/(10-(-10));  
    setMotorL(left_pulse);
}

void messageRightCb(const std_msgs::Float64 &msg) {
    int right_pulse=1200 + (msg.data-(-10))*1.0*(1800-1200)/(10-(-10));  
    setMotorR(right_pulse); 
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
