/*
 * Recibe la velocidad angular del motor izquierdo y derecho
 */

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

std_msgs::Float64 a;

void callback_function( const std_msgs::Float64MultiArray& toggle_msg){
  //Serial.println(toggle_msg.data[0]);
  a.data = toggle_msg.data[0];

  if (toggle_msg.data[0] > 3.3) {
    //digitalWrite(13, HIGH-digitalRead(13));   // blink
    digitalWrite(13, HIGH);   // blink
  }
  else {
    //digitalWrite(13, LOW-digitalRead(13));   // blink
    digitalWrite(13, LOW);   // blink
  }
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("vel_motor", callback_function);

std_msgs::Float64 float_msg;
ros::Publisher chatter("chatter", &float_msg);

void setup()
{
  //Serial.begin(4800);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  float_msg.data = a.data;
  //float_msg.data[1] = toggle_msg.data[1];
  chatter.publish( &float_msg);
  nh.spinOnce();
  delay(90);
}
