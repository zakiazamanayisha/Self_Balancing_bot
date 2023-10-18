#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

int16_t accX,accY,accZ,gyroX;
float roll_angle,pitch_angle,gyroAngle,currentAngle=0,previousAngle=0;
unsigned long currTime,prevTime=0,loopTime;

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

void loop()
{  
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  //acceleromter calculation 
  accX=a.acceleration.x;
  accY=a.acceleration.y;
  accZ=a.acceleration.z;

  roll_angle=atan(accY/sqrt(accX*accX+accZ*accZ))*1/(3.142/180);
  pitch_angle=-atan(accX/sqrt(accY*accY+accZ*accZ))*1/(3.142/180);


  

  Serial.print("acceleromter values : \n");
  Serial.print("x-axis:\n");
  Serial.println(accX);
  Serial.print("y-axis:\n");
  Serial.println(accY);
  Serial.print("z-axis:\n");
  Serial.println(accZ);

  Serial.print("\n angle values: \n");
  
  Serial.print("roll: \n");
  Serial.println(roll_angle);
  Serial.print(" ");
  Serial.print("pitch: \n");
  Serial.println(pitch_angle);
  Serial.print(" ");

  //gyro calculation 
  gyroX=g.gyro.x;
  gyroAngle=gyroAngle+(float)gyroX*loopTime/1000;
  Serial.print("\nGyro Angle:");
  Serial.println(gyroAngle);
  Serial.print(" ");
  
  //complementary filter
  currentAngle = 0.9934 * (previousAngle + gyroAngle) + 0.0066 *(pitch_angle);
  Serial.print("\nCurrent Angle: ");
  Serial.println(currentAngle);
  
  
  delay(1000);
}
