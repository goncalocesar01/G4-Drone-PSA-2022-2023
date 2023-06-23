#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>


MPU6050 mpu;

int numSamples = 1000;  // Número de amostras para a calibração

float accelXOffset = 0.0, accelYOffset = 0.0, accelZOffset = 0.0;
float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0;
float gyroZ_dpsoffset=0.0, accel_angle_xoffset=0.0,accel_angle_yoffset=0.0;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
float gamepadarray[5];

int JLX;
int JLY;
int JRX;
int JZ;
bool A;
// pin do sensor utlrasom
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
 
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

// PID controller constantes
float kp_roll = 1.2;   // Proportional gain for roll angle
float ki_roll = 0.004;   // Integral gain for roll angle
float kd_roll = 0.005;   // Derivative gain for roll angle

float kp_pitch = kp_roll ;  // Proportional gain for pitch angle
float ki_pitch = ki_roll;  // Integral gain for pitch angle
float kd_pitch = ki_roll;  // Derivative gain for pitch angle

float kp_yaw = 0;    // Proportional gain for yaw angle
float ki_yaw = 0.0;    // Integral gain for yaw angle
float kd_yaw = 0.00000;    // Derivative gain for yaw angle

float kp_thrust=0.0;
float ki_thrust=0.0;
float kd_thrust=0.0000;
float currentTime=0.0;
float previousTime = 0.0;
// Variables
float error, previous_error, pid_p, pid_i, pid_d, pid_roll;
float error1, previous_error1, pid_p1, pid_i1, pid_d1, pid_pitch;
float error2, previous_error2, pid_p2, pid_i2, pid_d2, pid_yaw;
float error3, previous_error3, pid_p3, pid_i3, pid_d3, pid_Thrust;
float elapsed_time = 0;  // Elapsed time in milliseconds

// Desired angles
float desired_roll_angle = 0.0;
float desired_pitch_angle = 0.0;
float desired_yaw_angle = 0.0;
float desiredthrottle=0.0;
// velocidade
float gas1,gas2,gas3,gas4;
long duration, distanceCm, previousdistance,velocidadeZ;

void calibrateMPU6050() {
  // Aguarde alguns segundos para que o sensor estabilize
  delay(2000);
  // Variáveis para armazenar as somas dos valores brutos
  int accelXSum = 0;
  int accelYSum = 0;
  int accelZSum = 0;
  int gyroXSum = 0;
  int gyroYSum = 0;
  int gyroZSum = 0;
  
  // Calcular erro de angulo
  for (int i = 0; i < 1000; i++) {
  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    accelXSum += ((atan((accelY)/sqrt(pow((accelX),2) + pow((accelZ),2)))*RAD_TO_DEG));
    accelYSum += ((atan(-1*(accelX)/sqrt(pow((accelY),2) + pow((accelZ),2)))*RAD_TO_DEG));
    gyroXSum += gyroX/32.8;
    gyroYSum += gyroY/32.8;
    gyroZSum += gyroZ/32.8;
    delayMicroseconds(100);  // Aguarde um curto intervalo entre as leituras
  }
  // Calcule a média dos valores brutos
  accelXOffset = accelXSum / 1000;
  accelYOffset = accelYSum / 1000;
  gyroXOffset = gyroXSum / 1000;
  gyroYOffset = gyroYSum / 1000;
  gyroZOffset = gyroZSum / 1000;
}

void setup() {
  Serial.begin(115200); 
  //leitura do sensor ultrassom
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);

  motor1.attach(12, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motor2.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motor3.attach(11, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motor4.attach(10, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  
  Wire.begin();
  mpu.initialize();
  mpu.setDMPEnabled(true);

  // Aguarde alguns segundos para que o MPU6050 seja inicializado
  delay(2000);

  // Realize a calibração
  calibrateMPU6050();
  calibrateangle();
}

void loop() {
  previousTime = currentTime;
  currentTime = millis() / 1000.0; // Convert milliseconds to seconds
  elapsed_time = currentTime - previousTime;
  //Serial.print("time=");
  //Serial.println(elapsed_time);
  // comunicação gamepad
  if (Serial.available()>= sizeof(gamepadarray)){
    String comando= Serial.readStringUntil('\n');
    sscanf(comando.c_str(),"%d,%d,%d,%d,%d", &JLX,&JLY,&JRX,&JZ,&A);
    }else {
    JLX=0;
    JLY=0;
    JRX=0;
    JZ=0;
    A=0;}

  int desiredroll= map(JLX,-255,255,-5,5);
  int desiredpitch=map(JLY,-255,255,-5,5);
  int desiredyaw=map(JRX,-255,255,-5,5);
  int desiredthrottle=map(JZ,-255,255,-5,5);
  
  //sensor utlrassom

  digitalWrite(TRIG_PIN,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN,HIGH);
  previousdistance= distanceCm;
  distanceCm = duration / 29.1 / 2 ; 
  if (distanceCm <= 0){
    Serial.println("Out of range");
    velocidadeZ=0.0;
  }else {
    velocidadeZ=(distanceCm-previousdistance)/elapsed_time;
    Serial.print(distanceCm);
    Serial.println("cm");
    //Serial.println(velocidadeZ);
    
    Serial.println();
  }

  Wire.begin();
  mpu.initialize();
  // Leitura dos dados brutos do MPU6050
  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  // Converter os valores para unidades de gravidade (g) e graus por segundo (°/s)
  float accelX_g = (((atan((accelY)/sqrt(pow((accelX),2) + pow((accelZ),2)))*RAD_TO_DEG)) - accelXOffset);
  float accelY_g = (((atan(-1*(accelX)/sqrt(pow((accelY),2) + pow((accelZ),2)))*RAD_TO_DEG)) - accelYOffset);
  float gyroX_dps = (gyroX/32.8 - gyroXOffset);
  float gyroY_dps = (gyroY/32.8 - gyroYOffset);
  float gyroZ_dps = (gyroZ/32.8 - gyroZOffset);
  //Serial.println(accelX_g);
  //Serial.println(accelY_g);
  //Serial.println(accelZ_g);
  // Convert raw accelerometer data to degrees

  float roll = 0.98(accel_angle_y+gyroX_dps)+0.02*accelY_g;
  float pitch = 0.98(accel_angle_X+gyroY_dps)+0.02*accelX_g;
  float yawRate=gyroZ_dps;

    // Exibir os valores convertidos
  //Serial.print("Accel (g): ");Serial.print(accelX_g);Serial.print("  ");Serial.print(accelY_g);Serial.print("  ");Serial.print(accelZ_g);
  //Serial.print("  |  Gyro (°/s): ");Serial.print(gyroX_dps);Serial.print("  ");Serial.print(gyroY_dps);Serial.print("  ");Serial.println(gyroZ_dps);
  Serial.println(roll);
  Serial.println(pitch);
  //Serial.println(yawRate);
  
    // PID for roll angle
  previous_error=error;
  error = desiredroll-roll ;
  pid_p = kp_roll * error;  // Proportional
  pid_i += (ki_roll * (error+previous_error))*0.005;  // Integral
  pid_d = kd_roll * ((error - previous_error) / elapsed_time);  // Derivative
  pid_roll = pid_p + pid_i + pid_d;
  //Serial.println(error);
  //Serial.println(pid_roll);

  // PID for pitch angle
  previous_error1=error1;
  error1 = desiredpitch-pitch;
  pid_p1 = kp_pitch * error1;  // Proportional
  pid_i1 +=(ki_pitch * (error1+previous_error1))*0.005; // Integral
  pid_d1 = kd_pitch * ((error1 - previous_error1) / elapsed_time);  // Derivative
  pid_pitch = pid_p1 + pid_i1 + pid_d1;
  //Serial.println(error1);
  //Serial.println(pid_pitch);
  // PID for yaw angle
  previous_error2=error2;
  error2 = desiredyaw - yawRate;
  pid_p2 = kp_yaw * error2;  // Proportional
  pid_i2 = pid_i2 + (ki_yaw * (error2+previous_error2)*0.005); // Integral
  pid_d2 = kd_yaw * ((error2 - previous_error2) / elapsed_time);  // Derivative
  pid_yaw = pid_p2 + pid_i2 + pid_d2;
  //Serial.println(error2);
  //Serial.println(pid_yaw);
    // PID for Thrust
  previous_error3=error3;
  error3 = desiredthrottle-velocidadeZ;
  pid_p3 = kp_thrust * error3;  // Proportional
  pid_i3 = pid_i3 + (ki_thrust * error3);  // Integral
  pid_d3 = kd_thrust * ((error3 - previous_error3) / elapsed_time);  // Derivative
  pid_Thrust = pid_p3 + pid_i3 + pid_d3;
  //Serial.println(error3);
  //Serial.println(pid_Thrust);
// controlar acelerador
  gas1 = 1400+pid_Thrust - pid_yaw - pid_pitch + pid_roll;   // right front
  gas2 = 1400+pid_Thrust + pid_yaw - pid_pitch - pid_roll;   // right back
  gas3 = 1400+pid_Thrust - pid_yaw + pid_pitch - pid_roll;   // left back
  gas4 = 1400+pid_Thrust + pid_yaw + pid_pitch + pid_roll;   // left front
  
// fixar limites de acelerador
  if (gas1 < 1020) {gas1 = 1020;}
  if (gas1 > 2000) {gas1 = 2000;}
  if (gas2 < 1020) {gas2 = 1020;}
  if (gas2 > 2000) {gas2 = 2000;}
  if (gas3 < 1020) {gas3 = 1020;}
  if (gas3 > 2000) {gas3 = 2000;}
  if (gas4 < 1020) {gas4 = 1020;}
  if (gas4 > 2000) {gas4 = 2000;}
  //Serial.println(gas1);
  //Serial.println(gas2);
  //Serial.println(gas3);
  //Serial.println(gas4);
  motor1.writeMicroseconds(gas1);
  motor2.writeMicroseconds(gas2);
  motor3.writeMicroseconds(gas3);
  motor4.writeMicroseconds(gas4);
}

