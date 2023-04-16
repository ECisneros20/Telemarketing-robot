//Libraies
#include <Servo.h>

//Motors
Servo rightMotor;
Servo leftMotor;

// Ultrasonic sensors
const int US[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// Infrared sensors
const int IR[8] = {A8, A9, A10, A11, A12, A13, A14, A15};
// Array of 16 sensors + bumpers
float sensor_data[17];

// Encoders
const int encoder[2]={2,19};

// Variables for odometry calculations
volatile double vel_lin_x = 0;
volatile double vel_ang_y = 0;
volatile double vel_ang_z = 0;
volatile double pos_lin_x = 0;
volatile double pos_lin_y = 0;
volatile double pos_ang_z = 0;

volatile unsigned long currTime = 0;
volatile unsigned long prevTime = 0;
volatile unsigned long delta = 0;
volatile unsigned long pulses = 0;

// variables for encoders calculationss
volatile double wR=0;
volatile double wL=0;
volatile unsigned long ticksR=0;
volatile unsigned long ticksL=0;
volatile unsigned long TimeBackup=0;


void setup() {
    Serial.begin(9600);
    // Setup of ultrasonic and infrared sensors
    for (int i=0; i<=7; i++) {
        pinMode(US[i],INPUT);
        pinMode(IR[i],INPUT);
    }
    // Setup of encoders
    for(int i=0;i<=1;i++)
      pinMode(encoder[i],INPUT);
    // Setup of motors (RC)
    rightMotor.attach(5);
    leftMotor.attach(6);
    //Setup of interruptions
    attachInterrupt(digitalPinToInterrupt(encoder[0]), encoderL, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoder[1]), encoderR, FALLING);
    
}

void loop() {
    //Set motors speed
    rightMotor.writeMicroseconds(1500);
    leftMotor.writeMicroseconds(1500);
    
    if(millis()-TimeBackup>=100){
      //Reads from encoders (10 Hz)
      wR=ticksR/(800)*2*PI;
      wL=ticksL/(800)*2*PI;
      ticksL=0;
      ticksR=0; 
      TimeBackup=millis();
      //Improve using time between tiks (future update)  
    }else{
      //Reads from US
      for (int i=0; i<=7; i++)
        sensor_data[i] = read_US(i);
      //Reads from IR 
      for (int i=8; i<=15; i++)
        sensor_data[i] = read_IR(i);      
    }
}

float read_IR(int sensor) {
    float raw = 0, distance = 0;
    raw = analogRead(IR[sensor]);
    distance = 13*pow(raw*0.0048828125,-1);
    return distance;
}

float read_US(int sensor) {
    float raw = 0, distance = 0;
    raw = analogRead(US[sensor]);
    distance = raw*0.5; 
    return distance;
}
void encoderL(){
  ticksL++;
}
void encoderR(){
  ticksR++;
}
