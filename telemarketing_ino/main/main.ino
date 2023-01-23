// Ultrasonic sensors
const int US[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// Infrared sensors
const int IR[8] = {A8, A9, A10, A11, A12, A13, A14, A15};

void setup() {
  for(int i = 0; i <= 7; i++){
    pinMode(US[i],INPUT);
    pinMode(IR[i],INPUT);
  }
}

void loop() {
}

float read_IR(int sensor) {
  float raw = 0,distance=0;
  
  for(int i=0;i<10;i++){
    raw=analogRead(IR[sensor]);
    distance+=13*pow(raw*0.0048828125,-1);
    delay(10);
  }
  return distance/10;
}

float read_US(int sensor){
  float raw=0,distance=0;
   
  for(int i=0;i<10;i++){
    raw=analogRead(US[sensor]);
    distance += raw*0.5;
    delay(10);
  }
  return distance/10;
}
