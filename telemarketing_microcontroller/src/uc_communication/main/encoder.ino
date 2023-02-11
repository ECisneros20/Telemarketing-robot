volatile double velocidad=0;
volatile unsigned long TiempoActual=0;
volatile unsigned long TiempoAnt=0;
volatile unsigned long delta=0;
volatile unsigned long pulsos=0;

void setup() {
  Serial.begin(9600);
  pinMode(2,INPUT);
  attachInterrupt(digitalPinToInterrupt(2), aea, FALLING);
}

void loop() {
  TiempoActual=millis();
  if(TiempoActual-TiempoAnt>=1000){
    Serial.println(String((double)pulsos*60/(1000*8))+" rpm");
    pulsos=0;
    TiempoAnt=TiempoActual;
  }
}
void aea(){
  pulsos++;
  }
