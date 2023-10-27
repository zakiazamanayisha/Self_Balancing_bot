#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <AccelStepper.h>
​
const float Kp = 1;
const float Kd = 0.05;
const float Ki = 40;
const float targetAngle = 0;
const float maxPid = 1000;
​
Adafruit_MPU6050 mpu;
​
//AccelStpper class instance
AccelStepper leftMotor(1, D3, D4); //(type of driver, step,dir)
AccelStepper rightMotor(1, D6, D7);
​
int16_t accX, accY, accZ, gyroX;
float motorPower;
float roll_angle = 0, pitch_angle = 0, gyroAngle = 0;
float currentAngle = 0, previousAngle = 0, error = 0 , prevError = 0, errorSum = 0;
uint32_t currTime, prevTime = 0, loopTime;
float iTerm, dTerm;
​
void setMotor(float motorPower) {
  //run left motor
  leftMotor.setSpeed(motorPower);
  leftMotor.runSpeed();
​
  //run right motor
  rightMotor.setSpeed(-motorPower);
  rightMotor.runSpeed();
​
}
​
void setup() {
  Serial.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
​
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
​
  leftMotor.setMaxSpeed(1000);
  rightMotor.setMaxSpeed(1000);
​
  Serial.println("");
}
​
void loop()
{
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
​
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
​
  /* acceleromter calculation */
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  roll_angle = atan(accY / sqrt(accX * accX + accZ * accZ)) * 1 / (3.142 / 180);
  pitch_angle = -atan(accX / sqrt(accY * accY + accZ * accZ)) * 1 / (3.142 / 180);
​
  /* gyro calculation */
  gyroX = g.gyro.x;
  gyroAngle = gyroAngle + (float)gyroX * loopTime / 1000;
​
  //complementary filter
  currentAngle = 0.98 * (currentAngle + gyroAngle * (loopTime / 1000)) + 0.02 * (pitch_angle);
​
  //setting up PID
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  //integral term
  iTerm = errorSum * loopTime / 1000;
  //differentiate term
  dTerm = (currentAngle - previousAngle) / (loopTime / 1000);
  //calculating output
  motorPower = (Kp * error) + (Ki * iTerm) + (Kd * dTerm);
  previousAngle = currentAngle;
​
  if (motorPower > maxPid) motorPower = maxPid;
  else if (motorPower < -maxPid) motorPower = -maxPid;
  
  motorPower = constrain(motorPower, -1000, 1000);
  setMotor(motorPower);
​
  char debugmsg[20];
  sprintf(debugmsg, "Gyro: %0.3f, Roll: %0.3f, Pitch: %0.3f, Current_Angle: %0.3f, Motor Power: %0.3f\n", 
          gyroAngle, roll_angle, pitch_angle, currentAngle, motorPower);
  Serial.printf(debugmsg);  
}
