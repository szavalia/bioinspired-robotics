#include <Servo.h>


//define position name and value
#define left 60
#define right 120
#define middle 90
#define closed 60
#define fullOpen 160
#define halfOpen 120

#define waitTime 750


class Motor{
  //Define pins
  int en;
  int backwardIn; //Backward
  int forwardIn; //Forward


  public: 
  Motor(int en, int backwardIn, int forwardIn)
  {
    this->en = en;
    this->backwardIn = backwardIn;
    this->forwardIn = forwardIn;
  }

  void forward(uint8_t power) {
    analogWrite(this->en, power);
    digitalWrite(this->forwardIn, HIGH);
    digitalWrite(this->backwardIn, LOW);
  }

  void backward(uint8_t power) {
    analogWrite(this->en, power);
    digitalWrite(this->forwardIn, LOW);
    digitalWrite(this->backwardIn, HIGH);
  }

  void stop(uint8_t power) {
    analogWrite(this->en, power);
    digitalWrite(this->forwardIn, LOW);
    digitalWrite(this->backwardIn, LOW);
  }
  

  void setup(){
    //Setting pins
    pinMode(this->en, OUTPUT);
    pinMode(this->backwardIn, OUTPUT);
    pinMode(this->forwardIn, OUTPUT);

    //Turn off motors inputs
    digitalWrite(this->backwardIn, LOW);
    digitalWrite(this->forwardIn, LOW);
  }


};

class Ldr
{
  //Define properties
  int readAnalogPin;

  public: 
  Ldr(int readAnalogPin)
  {
    this->readAnalogPin = readAnalogPin;
  }

  int calculateLight()
  {
    return analogRead(readAnalogPin);
  }


};

class DistanceSensor
{
  //Define properties
  int trigPin; //Black
  int echoPin; //Red
  
  public:
  DistanceSensor(int trigPin, int echoPin){
    this->trigPin = trigPin;
    this->echoPin = echoPin;
  }

  int calculateDistance(){
    digitalWrite(trigPin, LOW);
    // delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    // delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    // Calculating and returning the distance
    int distance = duration * 0.034 / 2;
    return distance;
  }

  void setup(){
    pinMode(this->trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(this->echoPin, INPUT); // Sets the echoPin as an Input
  }



};

  ///Motors
  ////Left motor
  Motor leftMotor(3, 5, 4);
  ///Right motor
  Motor rightMotor(9, 8, 7);
  const int baseSpeed = 440;
  int currentLeftSpeed;
  int currentRightSpeed;
  ///Ldrs
  ////Left ldr
  Ldr leftLdr(1);
  ////Right ldr
  Ldr rightLdr(2);

  ///Distance sensor
  DistanceSensor distanceSensor(11,12);

  const int maxDistance = 10;
  const int maxLighting = 300;
  const int baseLightning = 200;
  ///Update intervals
  int distanceSensorUpdateInterval = 50;
  int ldrUpdateInterval = 10;
  int eyeUpdateInterval = 3000;
  ///Last updates
  long distanceSensorLastUpdate = 0;
  long ldrLastUpdate = 0;
  long eyeLastUpdate = 0;

  //define name of the servo motors
  Servo upDownServo;
  Servo rightLeftServo;

  const int rightLeftPin = 10;
  const int upDownPin = 2;

void setup() {
  //Initialize robot and his components

  //Components

  ///Motors
  ////Left motor
  leftMotor.setup();
  ///Right motor
  rightMotor.setup();


  ///Distance sensor
  distanceSensor.setup();

  currentLeftSpeed = baseSpeed;
  currentRightSpeed = baseSpeed;

  //Eye
  ///define pin numbers of the servo motors
  upDownServo.attach(upDownPin);
  rightLeftServo.attach(rightLeftPin);

  Serial.begin(9600);


}

void loop() {
  //update distance sensor
    if((millis() - distanceSensorLastUpdate) > distanceSensorUpdateInterval)  // time to update distance sensor
    {
      Serial.println("Updating sensor");
      distanceSensorLastUpdate = millis();
      int distance = distanceSensor.calculateDistance();
      // Serial.print("Distance : ");
      // Serial.println(distance);
      if(distance<maxDistance){
        // leftMotor.backward(currentLeftSpeed);
        rightMotor.backward(currentRightSpeed);
        leftMotor.backward(currentLeftSpeed);
        delay(2000);
      }
      else{
        // leftMotor.forward(currentLeftSpeed);
        rightMotor.forward(currentRightSpeed);
        leftMotor.forward(currentLeftSpeed);
      }

    }

    // leftMotor.backward(currentLeftSpeed);
    // rightMotor.backward(currentRightSpeed);
    // leftMotor.forward(currentLeftSpeed);
    // rightMotor.forward(currentRightSpeed);
    

    //update Ldrs
    if((millis() - ldrLastUpdate) > ldrUpdateInterval)  // time to update ldrs
    {
      ldrLastUpdate = millis();
      int leftLight = leftLdr.calculateLight();
      int rightLight = rightLdr.calculateLight();
      Serial.print("Lights: Left light = ");
      Serial.println(leftLight);
      Serial.print("; Right light = ");
      Serial.println(rightLight);
      //Left motor update
      // if(leftLight > maxLighting){
        currentRightSpeed = updateSpeed(leftLight, baseSpeed, baseLightning);
      // }
      // if(rightLight > maxLighting){
        currentLeftSpeed = updateSpeed(rightLight, baseSpeed, baseLightning);
      // }
        Serial.print("currentLeftSpeed : ");
        Serial.println(currentLeftSpeed);
        Serial.print("currentRightSpeed : ");
        Serial.println(currentRightSpeed);

        int direction = currentRightSpeed - currentLeftSpeed > 0 ? left : right;
        rightLeftServo.write(direction);

    }

    if ((millis() - eyeLastUpdate) > eyeUpdateInterval){
      eyeLastUpdate = millis();
      upDownServo.write(closed);
      Serial.println(" Closing...");
      
    }

    if(  millis() > eyeLastUpdate + 200 ){
        upDownServo.write(fullOpen);
        Serial.println(" Opening...");
      }

    
    
}

int updateSpeed(int currentLightning, int baseSpeed, int baseLightning){

  // Serial.print("currentLightning : ");
  // Serial.println(currentLightning);
  int newSpeed = (int) (baseSpeed * (float) currentLightning / baseLightning);
  // Serial.print("Fraction: ");
  // Serial.println((float) currentLightning / baseLightning);

  // Serial.print("New speed : ");
  // Serial.println(newSpeed);
  return newSpeed;
}
