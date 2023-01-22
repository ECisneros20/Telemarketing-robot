void setup() {
  Serial.begin(9600);
  pinMode(13,INPUT);
  pinMode(A0,INPUT);
}

void loop() {
  int raw=pulseIn(13,HIGH);
  int raw2=analogRead(A0);
  Serial.print("ADC:"+String(raw2*0.5)+"cm-----------");
  Serial.println("Pulse:"+String(raw/10)+"cm");
  
  delay(100);
}
