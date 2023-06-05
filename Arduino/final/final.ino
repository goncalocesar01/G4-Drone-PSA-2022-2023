#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>


MPU6050 mpu;

int numSamples = 1000;  // Número de amostras para a calibração

float accelXOffset = 0.0, accelYOffset = 0.0, accelZOffset = 0.0;
float gyroXOffset = 0.0, gyroYOffset = 0.0, gyroZOffset = 0.0;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

float gamepadarray[5];

int JLX;
int JLY;
int JRX;
int JZ;
bool A;
// pin do sensor utlrasom
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
 

// PID controller constantes
float kp_roll = 3.00;   // Proportional gain for roll angle
float ki_roll = 0.05;   // Integral gain for roll angle
float kd_roll = 0.005;   // Derivative gain for roll angle

float kp_pitch = 3.55;  // Proportional gain for pitch angle
float ki_pitch = 0.05;  // Integral gain for pitch angle
float kd_pitch = 0.005;  // Derivative gain for pitch angle

float kp_yaw = 3.55;    // Proportional gain for yaw angle
float ki_yaw = 0.05;    // Integral gain for yaw angle
float kd_yaw = 0.005;    // Derivative gain for yaw angle

float kp_thrust=3.55;
float ki_thrust=0.05;
float kd_thrust=0.005;

// Variables
float error, previous_error, pid_p, pid_i, pid_d, pid_roll;
float error1, previous_error1, pid_p1, pid_i1, pid_d1, pid_pitch;
float error2, previous_error2, pid_p2, pid_i2, pid_d2, pid_yaw;
float error3, previous_error3, pid_p3, pid_i3, pid_d3, pid_Thrust;
float elapsed_time = 0.0000001;  // Elapsed time in milliseconds

// Desired angles
float desired_roll_angle = 0.0;
float desired_pitch_angle = 0.0;
float desired_yaw_angle = 0.0;

// velocidade
float gas1,gas2,gas3,gas4;

void setup() {
  
  
  Serial.begin(115200); 
  
  //leitura do sensor ultrassom
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);

  motor1.attach(12);
  motor2.attach(11);
  motor3.attach(10);
  motor4.attach(9);
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

}

void loop() {
  // comunicação gamepad
  if (Serial.available()>= sizeof(gamepadarray)){
    String comando= Serial.readStringUntil('\n');
    sscanf(comando.c_str(),"%d,%d,%d,%d,%d", &JLX,&JLY,&JRX,&JZ,&A);
    }else {
    JLX=0;
    JLY=0;
    JRX=0;
    JZ=-20;
    A=0;
  }

  int desiredroll= map(JLX,-255,255,-20,20);
  int desiredpitch=map(JLY,-255,255,-20,20);
  int desiredyaw=map(JRX,-255,255,-20,20);
  
  //sensor utlrassom
  long duration, distanceCm, distanceIn, previousdistance,velocidadeZ;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN,HIGH);
  distanceCm = duration / 29.1 / 2 ; 
  if (distanceCm <= 0)
  {
    Serial.println("Out of range");
  }
  else {
    velocidadeZ=(distanceCm-previousdistance)/elapsed_time;
    previousdistance= distanceCm;
    Serial.print(distanceCm);
    Serial.print("cm");
    Serial.println();

  }

  // Leitura dos dados brutos do MPU6050
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
  // Converter os valores para unidades de gravidade (g) e graus por segundo (°/s)
  float accelX_g = (accelX - accelXOffset) / 16384.0;
  float accelY_g = (accelY - accelYOffset) / 16384.0;
  float accelZ_g = (accelZ - accelZOffset) / 16384.0;
  float gyroX_dps = (gyroX - gyroXOffset) / 131.0;
  float gyroY_dps = (gyroY - gyroYOffset) / 131.0;
  float gyroZ_dps = (gyroZ - gyroZOffset) / 131.0;
  // Convert raw accelerometer data to degrees
  float roll = atan2(accelY_g, accelZ_g) * 180.0 / PI;
  float pitch = atan2(-accelX_g, sqrt(accelY_g*accelY_g + accelZ_g*accelZ_g)) * 180.0 / PI;
  float yawRate = (float)gyroZ_dps / 131.0;
    // Exibir os valores convertidos
  //Serial.print("Accel (g): ");Serial.print(accelX_g);Serial.print("  ");Serial.print(accelY_g);Serial.print("  ");Serial.print(accelZ_g);
  //Serial.print("  |  Gyro (°/s): ");Serial.print(gyroX_dps);Serial.print("  ");Serial.print(gyroY_dps);Serial.print("  ");Serial.println(gyroZ_dps);
  
  
    // PID for roll angle
  previous_error=error;
  error = roll - desiredroll;
  pid_p = kp_roll * error;  // Proportional
  pid_i += (ki_roll * error);  // Integral
  pid_d = kd_roll * ((error - previous_error) / elapsed_time);  // Derivative
  pid_roll = pid_p + pid_i + pid_d;
  // PID for pitch angle
  previous_error1=error1;
  error1 = pitch - desiredroll;
  pid_p1 = kp_pitch * error1;  // Proportional
  pid_i1 +=(ki_pitch * error1);  // Integral
  pid_d1 = kd_pitch * ((error1 - previous_error1) / elapsed_time);  // Derivative
  pid_pitch = pid_p1 + pid_i1 + pid_d1;
  // PID for yaw angle
  previous_error2=error2;
  error2 = yawRate - desiredyaw;
  pid_p2 = kp_yaw * error2;  // Proportional
  pid_i2 = pid_i2 + (ki_yaw * error2);  // Integral
  pid_d2 = kd_yaw * ((error2 - previous_error2) / elapsed_time);  // Derivative
  pid_yaw = pid_p2 + pid_i2 + pid_d2;
    // PID for Thrust
  
  previous_error3=error3;
  error3 = JZ-velocidadeZ;
  pid_p3 = kp_thrust * error3;  // Proportional
  pid_i3 = pid_i3 + (ki_thrust * error3);  // Integral
  pid_d3 = kd_thrust * ((error3 - previous_error3) / elapsed_time);  // Derivative
  pid_Thrust = pid_p3 + pid_i3 + pid_d3;

// controlar acelerador
  gas1 = 1300 + JZ + pid_yaw + pid_pitch + pid_roll;   // right front
  gas2 = 1300 + JZ - pid_yaw + pid_pitch - pid_roll;   // right back
  gas3 = 1300 + JZ - pid_yaw - pid_pitch + pid_roll;   // left back
  gas4 = 1300 + JZ + pid_yaw - pid_pitch - pid_roll;   // left front

// fixar limites de acelerador
  if (gas1 < 1000) {gas1 = 1000;}
  if (gas1 > 2000) {gas1 = 2000;}
  if (gas2 < 1000) {gas2 = 1000;}
  if (gas2 > 2000) {gas2 = 2000;}
  if (gas3 < 1000) {gas3 = 1000;}
  if (gas3 > 2000) {gas3 = 2000;}
  if (gas4 < 1000) {gas4 = 1000;}
  if (gas4 > 2000) {gas4 = 2000;}

  motor1.writeMicroseconds(gas1);
  motor2.writeMicroseconds(gas2);
  motor3.writeMicroseconds(gas3);
  motor4.writeMicroseconds(gas4);
  
}

void calibrateMPU6050() {
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;

  // Ler e descartar as primeiras leituras para aquecimento
  for (int i = 0; i < 200; i++) {
    mpu.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
    delay(10);
  }

  // Realizar a média das amostras para obter os offsets
  for (int i = 0; i < numSamples; i++) {
    mpu
.getMotion6(&accelX, &accelY, &accelZ, &gyroX, &gyroY, &gyroZ);
accelXOffset += accelX;
accelYOffset += accelY;
accelZOffset += accelZ;
gyroXOffset += gyroX;
gyroYOffset += gyroY;
gyroZOffset += gyroZ;
delay(10);
}

// Calcular a média dos offsets
accelXOffset /= numSamples;
accelYOffset /= numSamples;
accelZOffset /= numSamples;
gyroXOffset /= numSamples;
gyroYOffset /= numSamples;
gyroZOffset /= numSamples;
}

