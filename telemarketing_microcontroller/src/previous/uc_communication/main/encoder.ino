volatile double wR=0;
volatile double wL=0;
volatile unsigned long ticksR=0;
volatile unsigned long ticksL=0;
volatile unsigned long TimeBackup=0;

const int encoder[2]={2,3};
void setup() {
  for(int i=0;i<=1;i++)
    pinMode(encoder[i],INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder[0]), encoderL, FALLING);
  attachInterrupt(digitalPinToInterrupt(encoder[1]), encoderR, FALLING);
}

void loop() {
  
  if(millis()-TimeBackup>=100){ //100 ms  
    wR=ticksR/(800)*2*PI;
    wL=ticksL/(800)*2*PI;
    ticksL=0;
    ticksR=0;
    TimeBackup=millis();
  }
}
void encoderL(){
  ticksL++;
  }

void encoderR(){
  ticksR++;
  }