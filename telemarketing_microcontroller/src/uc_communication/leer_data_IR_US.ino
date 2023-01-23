#include <SharpIR.h>

/*
  2 to 15 cm GP2Y0A51SK0F  use 215
  4 to 30 cm GP2Y0A41SK0F / GP2Y0AF30 series  use 430
  10 to 80 cm GP2Y0A21YK0F  use 1080
  10 to 150 cm GP2Y0A60SZLF use 10150
  20 to 150 cm GP2Y0A02YK0F use 20150
  100 to 550 cm GP2Y0A710K0F  use 100550
*/
SharpIR SharpIR1(SharpIR::GP2Y0A41SK0F, A1);
SharpIR SharpIR2(SharpIR::GP2Y0A41SK0F, A2);
SharpIR SharpIR3(SharpIR::GP2Y0A41SK0F, A3);
SharpIR SharpIR4(SharpIR::GP2Y0A41SK0F, A4);
SharpIR SharpIR5(SharpIR::GP2Y0A41SK0F, A5);
SharpIR SharpIR6(SharpIR::GP2Y0A41SK0F, A6);

const int anPin8 = 8;
const int anPin9 = 9;
const int anPin10 = 10;
const int anPin11 = 11;
const int anPin12 = 12;
const int anPin13 = 13;
const int anPin14 = 14;
const int anPin15 = 15;


long distance1, distance2, distance3, distance4, distance5, distance6, distance7, distance8;

//SharpIR SharpIR7(SharpIR::GP2Y0A41SK0F, A7);

void setup() {
  // Multiple Sharp IR Distance meter code for Robojax.com
  Serial.begin(9600);
  Serial.println("Sharp IR  ");

}

void loop() {
  // Sharp IR code for Robojax.com 20181201


  unsigned long startTime = millis(); // takes the time before the loop on the library begins

  int dis1 = SharpIR1.getDistance(); // this returns the distance for sensor 1
  int dis2 = SharpIR2.getDistance(); // this returns the distance for sensor 2
  int dis3 = SharpIR3.getDistance(); // this returns the distance for sensor 1
  int dis4 = SharpIR4.getDistance(); // this returns the distance for sensor 2
  int dis5 = SharpIR5.getDistance(); // this returns the distance for sensor 1
  int dis6 = SharpIR6.getDistance(); // this returns the distance for sensor 2
  //  int dis7=SharpIR7.getDistance();  // this returns the distance for sensor 1

  Serial.print("Infrared sensors: ");
  Serial.print("Distance (1): ");
  Serial.print(dis1);
  Serial.print("cm" );

  Serial.print("Distance (2): ");
  Serial.print(dis2);
  Serial.print("cm ");

  Serial.print("Distance (3): ");
  Serial.print(dis3);
  Serial.print("cm ");

  Serial.print("Distance (4): ");
  Serial.print(dis4);
  Serial.print("cm ");

  Serial.print("Distance (5): ");
  Serial.print(dis5);
  Serial.print("cm ");

  Serial.print("Distance (6): ");
  Serial.print(dis6);
  Serial.println("cm ");
//
//  //  Serial.print("Distance (7): ");
//  //  Serial.print(dis7);
//  //  Serial.println("cm ");


  read_sensors();
  print_all();
//  delay(150);
}



/*depending on mounting and sensor environment chaining may not be required for these sensors
  The recommended mode of operation for these sensors is free-run mode.  It is recommended
  to test the sensors in free-run mode before chaining.
  If they free-run properly with minimal interference, the void loop delay can be reduced to 133
  and the section that says "start_sensor" can be commented out.*/


void read_sensors() {
  /*
    The Arduinoâ€™s analog-to-digital converter (ADC) has a range of 1024,
    which means each bit is ~4.9mV.
    Each bit is equal to 5mm so it needs to be multiplied by 5
  */
  distance1 = analogRead(anPin8) * 5;

  distance2 = analogRead(anPin9) * 5;

  distance3 = analogRead(anPin10) * 5;

  distance4 = analogRead(anPin11) * 5;

  distance5 = analogRead(anPin12) * 5;

  distance6 = analogRead(anPin13) * 5;

  distance7 = analogRead(anPin14) * 5;

  distance8 = analogRead(anPin15) * 5;

}

void print_all() {
  Serial.print("S1=");
  Serial.print(distance1);
  Serial.print("mm");
  delay(150);
  Serial.print(" S2=");
  Serial.print(distance2);
  Serial.print("mm");
  delay(150);
  Serial.print(" S3=");
  Serial.print(distance3);
  Serial.print("mm");
  delay(150);
  Serial.print(" S4=");
  Serial.print(distance4);
  Serial.print("mm");
  delay(150);
  Serial.print("S5=");
  Serial.print(distance5);
  Serial.print("mm");
  delay(150);
  Serial.print(" S6=");
  Serial.print(distance6);
  Serial.print("mm");
  delay(150);
  Serial.print(" S7=");
  Serial.print(distance7);
  Serial.print("mm");
  delay(150);
  Serial.print(" S8=");
  Serial.print(distance8);
  Serial.print("mm");
  Serial.println();
  delay(150);
}
