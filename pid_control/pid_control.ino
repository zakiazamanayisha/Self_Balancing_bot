#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <AccelStepper.h>

#define Kp 1
#define Kd 0.05
#define Ki 40
#define targetAngle 0
#define maxPid 1000

Adafruit_MPU6050 mpu;

//AccelStpper class instance 
AccelStepper leftMotor(1,D3,D4); //(type of driver, step,dir)
AccelStepper rightMotor(1,D6,D7);

int16_t accX,accY,accZ,gyroX;
float motorPower;
float roll_angle,pitch_angle,gyroAngle,currentAngle=0,previousAngle=0, error, prevError=0, errorSum=0;
unsigned long currTime,prevTime=0,loopTime;
float iTerm, dTerm;

void setMotor(float motorPower) 
{ 
  //run left motor
  leftMotor.setSpeed(motorPower);
  leftMotor.runSpeed();
  
  //run right motor
  rightMotor.setSpeed(-motorPower);
  rightMotor.runSpeed();

}

void setup() {  
  Serial.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
  // set maximum speed 
  leftMotor.setMaxSpeed(1000);
  rightMotor.setMaxSpeed(1000);
  
  Serial.println(" ");
  
}

void loop()
{  
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  motorPower = constrain(motorPower, -1000, 1000); 
  setMotor(motorPower);

  //acceleromter calculation 
  accX=a.acceleration.x;
  accY=a.acceleration.y;
  accZ=a.acceleration.z;

  roll_angle=atan(accY/sqrt(accX*accX+accZ*accZ))*1/(3.142/180);
  pitch_angle=-atan(accX/sqrt(accY*accY+accZ*accZ))*1/(3.142/180);

  //gyro calculation 
  gyroX=g.gyro.x;
  gyroAngle=gyroAngle+(float)gyroX*loopTime/1000;
  Serial.print("\nGyro Angle:");
  Serial.println(gyroAngle);
  Serial.print(" ");
  
  //complementary filter
  currentAngle = 0.98*(currentAngle+gyroAngle*(loopTime/1000)) + 0.02*(pitch_angle);
  Serial.print("\nCurrent Angle: ");
  Serial.println(currentAngle);

  //setting up PID
  error=currentAngle-targetAngle;
  errorSum=errorSum+error;
  //integral term
  iTerm=errorSum*loopTime/1000;
  //differentiate term
  dTerm=(currentAngle-previousAngle)/(loopTime/1000);
  //calculating output 
  motorPower= (Kp*error)+ (Ki*iTerm) + (Kd*dTerm);
  previousAngle=currentAngle;
  Serial.print("Motor Speed: ");
  Serial.println(motorPower);

  if (motorPower > maxPid) motorPower = maxPid;
  else if (motorPower < -maxPid) motorPower = -maxPid;
  
}
