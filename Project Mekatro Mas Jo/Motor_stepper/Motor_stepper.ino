
//Include the Arduino stepper library
#include <Stepper.h>

//Set how many steps it takes to make a full revolution
//Divide the degrees per step by 360 to get the steps
const int stepsPerRevolution = 2048;

//Use pin 8-11 to IN1-IN4

Stepper stepperName = Stepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {

  //Set the RPM of the stepper motor
  stepperName.setSpeed(5);

}

void loop() {

    //This should make the stepper do a full 360 degrees
    stepperName.step(2048);
    delay(5000);
    stepperName.step(0);
    delay(5000);
    
}