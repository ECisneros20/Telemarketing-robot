// Ultrasonic sensors
const int US[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// Infrared sensors
const int IR[8] = {A8, A9, A10, A11, A12, A13, A14, A15};
// Array of 16 sensors + bumpers
float sensor_data[17];

// Variables for odometry calculations
volatile double vel_lin_x = 0;
volatile double vel_ang_z = 0;
volatile double vel_ang_z = 0;
volatile double pos_lin_x = 0;
volatile double pos_lin_y = 0;
volatile double pos_ang_z = 0;

volatile double wR = 0;
volatile double wL = 0;
volatile unsigned long ticksR = 0;
volatile unsigned long ticksL = 0;
volatile unsigned long TimeBackup = 0;


volatile unsigned long currTime = 0;
volatile unsigned long prevTime = 0;
volatile unsigned long delta = 0;
volatile unsigned long pulses = 0;


void setup() {
    Serial.begin(9600);
    // Setup of ultrasonic and infrared sensors
    for (int i=0; i<=7; i++) {
        pinMode(US[i],INPUT);
        pinMode(IR[i],INPUT);
    }
}

void loop() {
    for (int i=0; i<=7; i++) {
        sensor_data[i] = read_US(i);
    }

    for (int i=8; i<=15; i++) {
        sensor_data[i] = read_IR(i);
    }
    Serial.print("US:");
    for (int i=0; i<=7; i++) {
        Serial.print((sensor_data[i]));
        Serial.print("---");
    }
    Serial.print("IR:");
    for (int i=8; i<=15; i++) {
        Serial.print((sensor_data[i]));
        Serial.print("---");
    }
    Serial.println("");

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
