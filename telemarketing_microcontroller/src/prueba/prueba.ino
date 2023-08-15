#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

ros::NodeHandle  nh;
Servo motorA;

std_msgs::Float32 a;

void callback_function( const std_msgs::Float32MultiArray& toggle_msg){
  //Serial.println(toggle_msg.data[0]);
  a.data = toggle_msg.data[0];

  if (toggle_msg.data[0] < 1.5) {
    //digitalWrite(13, HIGH-digitalRead(13));   // blink
    motorA.writeMicroseconds(1800);   // blink
  }
  else {
    //digitalWrite(13, LOW-digitalRead(13));   // blink
    motorA.writeMicroseconds(1500);  
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("vel_motors", callback_function);

std_msgs::Float32 float_msg;
ros::Publisher chatter("chatter", &float_msg);

void setup()
{
  //Serial.begin(4800);
  pinMode(13, OUTPUT);
  motorA.attach(5);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  pinMode(48,OUTPUT);
  digitalWrite(48,HIGH);
  delay(10);
  digitalWrite(48,LOW);
}

void loop()
{
  float_msg.data = a.data;
  //float_msg.data[1] = toggle_msg.data[1];
  chatter.publish( &float_msg);
  nh.spinOnce();
  delay(90);
}
