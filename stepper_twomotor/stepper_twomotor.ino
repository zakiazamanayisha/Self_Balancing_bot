#include <AccelStepper.h>


//AccelStpper class instance 

AccelStepper leftMotor(AccelStepper::DRIVER,D3,D4); //(type of driver, step,dir)
AccelStepper rightMotor(AccelStepper::DRIVER,D6,D7);

 void setup()
 {
  // set maximum speed 
  leftMotor.setMaxSpeed(1000);
  rightMotor.setMaxSpeed(1000);
 }

void loop() 
{ 
  //run left motor
  leftMotor.setSpeed(800);
  leftMotor.runSpeed();
  
  //run right motor
  rightMotor.setSpeed(-800);
  rightMotor.runSpeed();

  
}
