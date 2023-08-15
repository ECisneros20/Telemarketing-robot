#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <Servo.h>
//Bumper signals
const int reset=48;
const int aviso=50;
// Infrared sensors
const int IR[8] = {7, 6, 4, 0, 1, 2, 3, 5};
// Ultrasonic sensors
const int US[8] = {9, 11, 14, 12, 8, 10, 13, 15};
// Encoder ticks
volatile int ticksR=0;
volatile int ticksL=0;
const int encoderR[2]={3,2}; //{channelA, channelB}
const int encoderL[2]={19,18}; //{channelA, channelB}
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
// Array of 16 sensors + 2 encoders + 1 bumpers
float sensor_data[19];

//Motors
Servo MotorR;
Servo MotorL;

volatile unsigned long TimeBackup=0;
ros::NodeHandle nh;
std_msgs::Float32MultiArray msg;
ros::Publisher pub("/sensor_data", &msg);

void messageBool(const std_msgs::Bool& emergReset) {
   digitalWrite(reset, emergReset.data);
    }
// ROS subscriber    
ros::Subscriber<std_msgs::Bool> sub_reset("/reset", &messageBool);

void messageCb(const std_msgs::Float32MultiArray& vel_msg) {
    setMotorR(vel_msg.data[0]);
    setMotorL(vel_msg.data[1]);   
    }
// ROS subscriber
ros::Subscriber<std_msgs::Float32MultiArray> sub("/vel_motors", &messageCb);


void setup()
{ Serial.begin(57600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(pub);
  nh.subscribe(sub);
  nh.subscribe(sub_reset);
  for(int i=0;i<=1;i++){
    pinMode(encoderR[i],INPUT);
    pinMode(encoderL[i],INPUT);}
  MotorR.attach(5);
  MotorL.attach(6);
  attachInterrupt(digitalPinToInterrupt(encoderL[0]), encoderLcallback, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderR[0]), encoderRcallback, FALLING);
  //Bumper signals
   pinMode(reset, OUTPUT);
   pinMode(aviso, INPUT);
   digitalWrite(reset,LOW);
   digitalWrite(reset,HIGH); 
   setMotorR(0);
   setMotorL(0);                                                                                 
}

void loop()
{ for (int i=0; i<=7; i++) {
      sensor_data[i] = read_US(i);
  }

  for (int i=8; i<=15; i++) {
      sensor_data[i] = read_IR(i-8);
  }
  if(millis()-TimeBackup>=100){ //100 ms  
    
    sensor_data[16]=ticksL;
    sensor_data[17]=ticksR;

    //Set Bumper
    if (digitalRead(aviso)==HIGH){
      sensor_data[18]=1; //Motors Activate
    }

    else{
      sensor_data[18]=0; //Motors Deactivate
    }

    // Set the message data
    msg.data_length = 19;
    msg.data = sensor_data;
  
    // Publish the message
    pub.publish(&msg);
    //nh.spinOnce();
    TimeBackup=millis();
  }
  nh.spinOnce();
}

float read_IR(int sensor) {
    float raw = 0, distance = 0;

    /*for(int i=0; i<10; i++) {
        raw = analogRead(IR[sensor]);
        distance += 13*1.0/(raw*0.0049);
    }*/
    raw = analogRead(IR[sensor]);
    distance = 13*1.0/(raw*0.0049);
    return distance/100;
}

float read_US(int sensor) {
    float raw = 0, distance = 0;

    /*for(int i=0; i<10; i++) {
        raw = analogRead(US[sensor]);
        distance += raw*0.5;
    }*/
    raw = analogRead(US[sensor]);
    distance = raw*0.5;

    return distance/100;
}

void encoderLcallback(){
  if (digitalRead(encoderL[0]) != digitalRead(encoderL[1])){ 
    if (ticksL == encoder_maximum) {
      ticksL = encoder_minimum;
    }
    else {
      ticksL++;  
    }
    }
  else{
    if (ticksL == encoder_minimum) {
      ticksL = encoder_maximum;
    }
    else {
      ticksL--;  
    }
    } 
}

void encoderRcallback(){
  if (digitalRead(encoderR[0]) == digitalRead(encoderR[1])){ 
    if (ticksR == encoder_maximum) {
      ticksR = encoder_minimum;
    }
    else {
      ticksR++;  
    }
    }
  else{
    if (ticksR == encoder_minimum) {
      ticksR = encoder_maximum;
    }
    else {
      ticksR--;  
    }
    } 
}

void setMotorR(int pulse){
  if(pulse>0)
    pulse=map(pulse,0,2.92,1550,1600);
  else
    pulse=map(pulse,-2.92,0,1400,1450);
  MotorR.writeMicroseconds(pulse);
}

void setMotorL(int pulse){
  if(pulse>0)
    pulse=map(pulse,0,2.92,1550,1600);
  else
    pulse=map(pulse,-2.92,0,1400,1450);
  MotorL.writeMicroseconds(pulse);
}
