#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>
//Bumper signals
const int reset=48;
const int aviso=50;
// Infrared sensors
const int IR[8] = {7, 6, 4, 0, 1, 2, 3, 5};
// Ultrasonic sensors
//const int US[8] = {8, 9, 10, 11, 12, 13, 14, 15};
const int US[8] = {9, 11, 14, 12, 8, 10, 13, 15};
// Encoder ticks
volatile int ticksR=0;
volatile int ticksL=0;
const int encoderR[2]={3,2}; //{channelA, channelB}
const int encoderL[2]={19,18}; //{channelA, channelB}
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
// Array of 16 sensors + bumpers
float sensor_data[19];

//Motors
Servo MotorR;
Servo MotorL;

volatile unsigned long TimeBackup=0;
ros::NodeHandle nh;

std_msgs::Float32MultiArray msg;
ros::Publisher pub("/sensor_data", &msg);

void setup()
{ nh.initNode();
  nh.advertise(pub);
  for(int i=0;i<=1;i++){
    pinMode(encoderR[i],INPUT);
    pinMode(encoderL[i],INPUT);}
  MotorR.attach(5);
  MotorL.attach(6);
  attachInterrupt(digitalPinToInterrupt(encoderL[0]), encoderLcallback, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoderR[0]), encoderRcallback, FALLING);

  //
   pinMode(reset, OUTPUT);
   pinMode(aviso, INPUT);

   digitalWrite(reset,HIGH);
   digitalWrite(reset,LOW);
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
    
    // Set the message data
    msg.data_length = 18;
    msg.data = sensor_data;
  
    // Publish the message
    pub.publish(&msg);
    nh.spinOnce();
    TimeBackup=millis();
  }
  
}

float read_IR(int sensor) {
    float raw = 0, distance = 0;

    for(int i=0; i<10; i++) {
        raw = analogRead(IR[sensor]);
        distance += 13*pow(raw*0.0048828125,-1);
        delay(10);
    }

    return distance/1000;
}

float read_US(int sensor) {
    float raw = 0, distance = 0;

    for(int i=0; i<10; i++) {
        raw = analogRead(US[sensor]);
        distance += raw*0.5;
        delay(10);
    }

    return distance/1000;
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
    pulse=map(pulse,0,0.3,1550,1600);
  else
    pulse=map(pulse,-0.3,0,1400,1450);
  MotorR.writeMicroseconds(pulse);
}

void setMotorL(int pulse){
  if(pulse>0)
    pulse=map(pulse,0,0.3,1550,1600);
  else
    pulse=map(pulse,-0.3,0,1400,1450);
  MotorL.writeMicroseconds(pulse);
}
