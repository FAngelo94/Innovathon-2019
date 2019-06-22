#include <PID_v1.h>

// PID stuff
double Setpoint, Input, Output;
PID constroller(&Input, &Output, &Setpoint, 10, 0, 0, DIRECT);

// Sonar stuff
int trigPin = 11;    // Trigger
int echoPin = 12;    // Echo
long duration, cm, inches;

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  
  //Sonar setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //PID setup
  Setpoint = 10;

  //turn the PID on
  constroller.SetMode(AUTOMATIC);
  constroller.SetOutputLimits(-255, 255);
}

void loop() {
  readDistance();
  Serial.print("distance: ");
  Serial.print(cm);
  Serial.print(" - ");

  Input = cm;
  constroller.Compute();
  Serial.print("output: ");
  Serial.print(Output);
  Serial.println();
  
  delay(250);
}

void readDistance(){
    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
   
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
   
    // Convert the time into a distance
    cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
}
