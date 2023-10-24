#include <Servo.h>

//Tutorial link: https://github.com/XRobots/ServoSmoothing/blob/main/code/sequencer/sequencer.ino

int pot2 = 1000;
int pot4 = 1000;

float pot2Scaled;
float pot4Scaled;
float pot4aScaled;


float pot2Smoothed = 512;
float pot4Smoothed = 512;

float pot2SmoothedPrev = 512;
float pot4SmoothedPrev = 512;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 5;        // time constant for timer

int stepFlag = 0;
long previousStepMillis = 0;

//define name of the servo motors
Servo upDownServo;
Servo rightLeftServo;

const int rightLeftPin = 10;
const int upDownPin = 2;

void setup() {


  Serial.begin(115200);

  //define pin numbers of the servo motors
  upDownServo.attach(upDownPin);
  rightLeftServo.attach(rightLeftPin);

  upDownServo.writeMicroseconds(1500);

  rightLeftServo.writeMicroseconds(1550);

}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  // start 5ms timed loop  
      previousMillis = currentMillis;

      // step sequencer

      if (stepFlag == 0 && currentMillis - previousStepMillis > 500) {
        stepFlag = 1;           
        previousStepMillis = currentMillis;
      }

      else if (stepFlag == 1 && currentMillis - previousStepMillis > 1000) {
        pot2 = 1024;
        stepFlag = 2;           
        previousStepMillis = currentMillis;
      }

      else if (stepFlag == 2 && currentMillis - previousStepMillis > 1500) {
        pot2 = 0;
        stepFlag = 3;           
        previousStepMillis = currentMillis;
      }

      if (stepFlag == 3 && currentMillis - previousStepMillis > 1500) {
        pot2 = 512;
        stepFlag = 4;           
        previousStepMillis = currentMillis;
      }

      if (stepFlag == 4 && currentMillis - previousStepMillis > 1500) {
        pot4 = 0;
        stepFlag = 5;           
        previousStepMillis = currentMillis;
      }
      if (stepFlag == 5 && currentMillis - previousStepMillis > 1500) {
        pot4 = 1024;
        stepFlag = 1;           
        previousStepMillis = currentMillis;
      }

      // end of step sequencer

      
    
      // scale all pots for the servo microseconds range
    
      pot2Scaled = (pot2 - 512) + 1500;
      pot4Scaled = ((pot4 - 512) * 1.2) + 1500;
    
      pot4Scaled = constrain(pot4Scaled,1100,2500);
    
      // smooth pots
      
      pot2Smoothed = (pot2Scaled * 0.01) + (pot2SmoothedPrev * 0.99);
      pot4Smoothed = (pot4Scaled * 0.05) + (pot4SmoothedPrev * 0.95);
    
      // bookmark previous values
    
      pot2SmoothedPrev = pot2Smoothed;
      pot4SmoothedPrev = pot4Smoothed;
  
      Serial.print(pot2Smoothed);
      Serial.print(" , ");
      Serial.print(pot4Smoothed);
      Serial.print(" , ");

      // write servos
            
      rightLeftServo.writeMicroseconds(pot2Smoothed);               //eyeballs side-side
    
      upDownServo.writeMicroseconds(pot4Smoothed);                  //eyelids

  } // end of timed loop


} // end if main loop