#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

int16_t accY, accZ;
float accAngle;
int16_t gyroX, gyroRate;
float gyroAngle=0;
unsigned long currTime, prevTime=0, loopTime;


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
}

void loop() {  
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accZ=a.acceleration.z;
  accY=a.acceleration.y;

  
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;

  gyroX = g.gyro.x;
  //gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroX*loopTime/1000;
  
  Serial.print("Gyro Angle: ");
  Serial.println(gyroAngle);
  
  if(isnan(accAngle));
  else
  {
    Serial.print("Accel Angle : ");
    Serial.println(accAngle);
  }
    delay(1000);
}
