#include <Arduino.h>
#include <Servo.h>

#define SENSOR_PIN A5
#define SERVO_PIN 11

Servo myServo;
const int N_SAMPLES = 10;
const float ALPHA = 40.0f / 93.0f; 
const float BETA  = 780.0f / 93.0f; 
float Kp = 8;  
float Ki = 0.5; 
float Kd = 5;  
float setpoint = 35.0;   // target distance (cm
float integral = 0.0;
float lastError = 0.0;
unsigned long lastTime = 0;
const int servoCenter = 90; 
const int servoMin = 80;  
const int servoMax = 100; 
float filteredOutput = 0.0;
const float smoothConst = 0.6;
int lastServoPos = servoCenter;
void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  myServo.write(servoCenter); 
  lastTime = millis();
}
float getAverageVoltage() {
  long sum = 0;
  for (int i = 0; i < N_SAMPLES; i++) {
    sum += analogRead(SENSOR_PIN);
    delay(2);
  }
  float avgRaw = sum / (float)N_SAMPLES;
  return avgRaw * (5.0f / 1023.0f);
}

void loop() {
  float voltage = getAverageVoltage();
  float d_meas = 27.86f / (voltage - 0.42f);
  float d_true = ALPHA * d_meas + BETA;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float error = setpoint - d_true;
  integral += error * dt;
  float derivative = (error - lastError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  filteredOutput = (smoothConst * output) + (1.0 - smoothConst) * filteredOutput;
  int targetServoPos = servoCenter + (int)filteredOutput;
  targetServoPos = constrain(targetServoPos, servoMin, servoMax);
  if (abs(targetServoPos - lastServoPos) > 5) { 
    if (targetServoPos > lastServoPos) lastServoPos += 5;
    else lastServoPos -= 5;
  } else {
    lastServoPos = targetServoPos;
  }

  myServo.write(lastServoPos);
  delay(30); 
}
