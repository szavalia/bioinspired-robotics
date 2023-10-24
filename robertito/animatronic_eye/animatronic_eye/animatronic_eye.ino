//include library for servo
#include <Servo.h>

//define name of the servo motors
Servo upDownServo;
Servo rightLeftServo;

//define position name and value
#define left 60
#define right 120
#define middle 90
#define closed 60
#define fullOpen 160
#define halfOpen 120

#define waitTime 750

const int rightLeftPin = 10;
const int upDownPin = 2;

void setup(){
  //define pin numbers of the servo motors
  upDownServo.attach(upDownPin);
  rightLeftServo.attach(rightLeftPin);

  //starting position of the servo motors
  delay(10);
  upDownServo.write(closed);
  rightLeftServo.write(middle);

   Serial.begin(9600);
}

void loop(){
  
  delay(1000);
  upDownServo.write(halfOpen);
  // Serial.print("Up-down : ");
  // Serial.println(halfOpen);
  delay(waitTime);
  rightLeftServo.write(right);
  // Serial.print("Right-left : ");
  // Serial.println(right);
  delay(waitTime);
  rightLeftServo.write(left);
  // Serial.print("Right-left : ");
  // Serial.println(left);
  delay(waitTime);
  rightLeftServo.write(middle);
  // Serial.print("Right-left : ");
  // Serial.println(middle);

  delay(1000);
  upDownServo.write(closed);
  // Serial.print("Up-down : ");
  // Serial.println(closed);
  delay(waitTime);
  upDownServo.write(fullOpen);
  // Serial.print("Up-down : ");
  // Serial.println(fullOpen);
  delay(waitTime);
  upDownServo.write(closed);
  // Serial.print("Up-down : ");
  // Serial.println(closed);
  delay(waitTime);


  upDownServo.write(fullOpen);
  // Serial.print("Up-down : ");
  // Serial.println(fullOpen);
  delay(waitTime);
  rightLeftServo.write(right);
  // Serial.print("Right-left : ");
  // Serial.println(right);
  delay(waitTime);
  rightLeftServo.write(left);
  // Serial.print("Right-left : ");
  // Serial.println(left);
  delay(waitTime);
  rightLeftServo.write(middle);
  upDownServo.write(halfOpen);
  delay(waitTime);
  rightLeftServo.write(right);
  delay(waitTime);
  rightLeftServo.write(left);
  delay(waitTime);
  rightLeftServo.write(middle);
  delay(waitTime);
  upDownServo.write(fullOpen);
  delay(waitTime);
  upDownServo.write(halfOpen);
  delay(waitTime);
  upDownServo.write(fullOpen);
  delay(waitTime);
  rightLeftServo.write(right);
  delay(waitTime);
  rightLeftServo.write(left);
  delay(waitTime);
  rightLeftServo.write(middle);
}
