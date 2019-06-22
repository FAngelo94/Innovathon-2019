#include <PID_v1.h>

const int loop_delay = 0;
int turn_right_num = 0;
// START
bool go = false;   //False in partenza, True al verde del semaforo
const int RECV_PIN = A2;

// SETUP VELOCITA
int max_power = 180; // Potenza massima motori da usare
float v_max = 0.5;   // Percentuale di potenza per movimento rettilineo

//PID parameter
const double k_p = 12;
const double k_i = 0;
const double k_d = 0;
double pid_setpoint = 10;
double pid_in, pid_out;
double max_out = 75;
PID constroller(&pid_in, &pid_out, &pid_setpoint, k_p, k_i, k_d, DIRECT);

// PIN per i MOTORI
// motor right
const int enR =  10;
const int in1 =  9;
const int in2 =  8;
// motor left
const int enL = 5;
const int in3 = 6;
const int in4 = 7;

// Sonar
const int trigPin = 11;
const int echoPin = 12;
long distance; //distance in cm


// Infrarossi
const int vout = 0;


void setup() {  
  set_arduino_pin();
  
  // PID setup
  constroller.SetMode(AUTOMATIC);
  constroller.SetOutputLimits(-max_out, max_out);

  // turn on motor RIGHT
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enR, 0);
  
  // turn on motor LEFT
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enL, 0);

  //For test
  //Serial.begin(9600);
}

double y;
long oldTime = 0;
long mytime;

void loop() {
  if(go){
    readDistance();
    if(distance > 38 && turn_right_num==0){
      turn_right();
      turn_right_num = 250;
    }
    else{
      if(turn_right_num > 0){
        turn_right_num--;
      }
      y = get_controller_output();
      set_power(y);
    }
    delay(loop_delay);
  }else{
    check_start();
  }
  //delay(1000);
  //mytime = millis();
  //prints time since program started
  //Serial.println(mytime-oldTime);
  //oldTime = mytime;
}

void check_start(){
  int valore=analogRead(RECV_PIN);
  float voltage = valore * (5.0 / 1023.0);
  if(voltage<2){
    go = true; 
    starting_line();
  }
}

void set_arduino_pin(){
  //Start
  pinMode(RECV_PIN,INPUT);
  
  // Sonar setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void readDistance(){
    int count = 0;
    long duration;
    long minDistance = 10000;
    while(count<1){
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
      duration = pulseIn(echoPin, HIGH, 20000);
      
      // Convert the time into a distance
      distance = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
      if(distance < minDistance){
        minDistance = distance;
      }
      count++;
      delay(10);
    }
    distance = minDistance;
    if(distance == 0)
      distance = 40;
}

double get_controller_output(){
  pid_in = distance;
  constroller.Compute();
  return pid_out / max_out;
}

void set_power(double y)
{
  int p_rx; //potenza motore dx
  int p_lx; //potenza motore sx
  p_rx = max_power * v_max - 75 * y;//(1 - v_max) * y * max_power;
  p_lx = max_power * v_max + 75 * y;//(1 - v_max) * y * max_power;
  
  if(p_rx < 0)
    p_rx = 0;
  if(p_lx < 0)
    p_lx = 0;
  
  if(p_rx > 255)
    p_rx = 255;
  if(p_lx > 255)
    p_lx = 255;
  
  
  //Serial.print("DISTANCE=");
  //Serial.print(distance);
  //Serial.print(" Y=");
  //Serial.print(y);
  //Serial.print(" L=");
  //Serial.print(p_lx);
  //Serial.print(" - ");
  //Serial.print("R=");
  //Serial.println(p_rx);
  
  analogWrite(enR, p_rx);
  analogWrite(enL, p_lx);
}

void turn_right(){
  analogWrite(enR, 0);
  analogWrite(enL,  0);
  delay(100);
  analogWrite(enL,  max_power * v_max);
  delay(700);
  analogWrite(enL,  0);
  delay(100);
}

void starting_line(){
  analogWrite(enL,  max_power * v_max);
  analogWrite(enR,  max_power * v_max);
  delay(1000);
}
