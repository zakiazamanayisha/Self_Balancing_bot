#include <AccelStepper.h>


//AccelStpper class instance 

AccelStepper leftMotor(AccelStepper::DRIVER,D3,D4); //(type of driver, step,dir)
//AccelStepper rightMotor(AccelStepper::DRIVER,D6,D7);


void setMotor(float motorPower)
{
  //run left motor
  leftMotor.setSpeed(motorPower);
  leftMotor.runSpeed();
  
  //run right motor
  //rightMotor.setSpeed(motorPower);
  //rightMotor.runSpeed();

}

 void setup()
 {
  // set maximum speed 
  leftMotor.setMaxSpeed(1000);
  //rightMotor.setMaxSpeed(1000);

  leftMotor.setAcceleration(10);
  //rightMotor.setAcceleration(10);

  
 }

void loop() 
{ 

  setMotor(1000);
}
