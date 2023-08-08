#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>

// Motors
Servo MotorR;
Servo MotorL;

const int reset=48;

void messageCb(const std_msgs::Int32MultiArray& msg);

ros::NodeHandle nh;

ros::Subscriber<std_msgs::Int32MultiArray> sub("/servo_vel", &messageCb);

void setup() {
    Serial.begin(57600);
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
    pinMode(reset,OUTPUT);
    MotorR.attach(5);
    MotorL.attach(6);
    digitalWrite(reset,LOW);
   digitalWrite(reset,HIGH); 
   setMotorR(1500);
   setMotorL(1500);
}

void loop() {
    nh.spinOnce();
}

void setMotorR(int pulse){
    MotorR.writeMicroseconds(pulse);
}

void setMotorL(int pulse){
    MotorL.writeMicroseconds(pulse);
}

// Definición de la función messageCb
void messageCb(const std_msgs::Int32MultiArray& msg) {
    setMotorR(msg.data[0]); // Corregir el acceso al valor en el mensaje
    setMotorL(msg.data[1]); // Corregir el acceso al valor en el mensaje
}
