#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
// Ultrasonic sensors
const int US[8] = {0, 1, 2, 3, 4, 5, 6, 7};
// Infrared sensors
const int IR[8] = {8, 9, 10, 11, 12, 13, 14, 15};
// Encoder ticks
volatile int ticksR=0;
volatile int ticksL=0;
const int encoder[2]={2,3};
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
// Array of 16 sensors + bumpers
float sensor_data[19];

volatile unsigned long TimeBackup=0;
ros::NodeHandle nh;

std_msgs::Float32MultiArray msg;
ros::Publisher pub("/sensor_data", &msg);

void setup()
{ nh.initNode();
  nh.advertise(pub);
  for(int i=0;i<=1;i++)
    pinMode(encoder[i],INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder[0]), encoderL, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder[1]), encoderR, FALLING);
}

void loop()
{ for (int i=0; i<=7; i++) {
      sensor_data[i] = read_US(i);
  }

  for (int i=8; i<=15; i++) {
      sensor_data[i] = read_IR(i);
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

void encoderL(){
  if (ticksL == encoder_maximum) {
    ticksL = encoder_minimum;
  }
  else {
    ticksL++;  
  } 
}

void encoderR(){
  if (ticksR == encoder_maximum) {
    ticksR = encoder_minimum;
  }
  else {
    ticksR++;  
  } 
}
