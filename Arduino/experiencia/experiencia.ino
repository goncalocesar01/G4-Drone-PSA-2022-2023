#include <Servo.h>
Servo ESC1; // create servo object to control the ESC
Servo ESC2;
Servo ESC3;
Servo ESC4;

int motor1=12;
int motor2=11;
int motor3=10; 
int motor4=9;
int a=180;
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
 
void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);
  // Attach the ESC on pin 9
  ESC1.attach(motor1,1000,2000);
  ESC2.attach(motor2,1000,2000);
  ESC3.attach(motor3,1000,2000);
  ESC4.attach(motor4,1000,2000); //(pin, min pulse width, max pulse width in microseconds) 
}
void loop()
{
  long duration, distanceCm, distanceIn;
 
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN,HIGH);
  // convert the time into a distance
  distanceCm = 4;
  distanceIn = duration / 74 / 2;
  
  if (distanceCm <= 0|distanceCm>=50){
    ESC1.write(0); 
    ESC2.write(0);
    ESC3.write(0); 
    ESC4.write(0);
  }
  else{
    Serial.println(distanceCm);
    int servocnv=180;
    ESC1.write(a-servocnv); 
    ESC2.write(a-servocnv);
    ESC3.write(a-servocnv); 
    ESC4.write(a-servocnv);
  }
}
