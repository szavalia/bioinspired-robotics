class Motor{
  //Define pins
  int en;
  int backwardIn; //Backward
  int forwardIn; //Forward
  int interval;
  // Define directions
  const int FORWARD = 1;
  const int BACKWARD = 2;
  const int STILL = 0;


  public: 
  Motor(int backwardIn, int forwardIn)
  {
    this->backwardIn = backwardIn;
    this->forwardIn = forwardIn;
  }
  
  void activateMotor(uint8_t power, uint8_t direction) {
    analogWrite(en, power);
    switch(direction){
      case FORWARD:
        digitalWrite(in2, HIGH);
        digitalWrite(in1, LOW);
        break;
      case BACKWARD:
        digitalWrite(in2, LOW);
        digitalWrite(in1, HIGH);
        break;
      case STILL:
        digitalWrite(in2, LOW);
        digitalWrite(in1, LOW);
        break;
    }


  }


}

class Ldr
{

}

class Robot
{
  //Define properties
  ///Motors
  Motor leftMotor;
  Motor rightMotor;
  ///LDRs
  Ldr leftLdr;
  Ldr rightLdr;
  ///Distance sensor
  DistanceSensor distanceSensor;
  ///Update intervals
  int distanceSensorUpdateInterval;
  int ldrUpdateInterval;
  ///Last updates
  long distanceSensorLastUpdate;
  long ldrLastUpdate;


  public: 
  Robot(Motor leftMotor, Motor rightMotor, Ldr leftLdr, Ldr rightLdr, DistanceSensor distanceSensor, int distanceSensorUpdateInterval, int ldrUpdateInterval)
  {
    ///Motors
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    ///Ldrs
    this->leftLdr = leftLdr;
    this->rightLdr = rightLdr;
    ///Distance sensor
    this->distanceSensor = distanceSensor;
    ///Update intervals
    this->distanceSensorUpdateInterval = distanceSensorUpdateInterval;
    this->ldrUpdateInterval = ldrUpdateInterval;
  }

  void Update()
  {
    //update distance sensor
    if((millis() - this->distanceSensorLastUpdate) > this->distanceSensorUpdateInterval)  // time to update distance sensor
    {
      this->distanceSensorLastUpdate = millis();
      // pos += increment;
      // servo.write(pos);
      // Serial.println(pos);
      // if ((pos >= 180) || (pos <= 0)) // end of sweep
      // {
      //   // reverse direction..................................................................................000
      //   increment = -increment;
      // }
    }

    //update Ldrs
    if((millis() - this->ldrLastUpdate) > this->ldrUpdateInterval)  // time to update ldrs
    {
      this->ldrLastUpdate = millis();
      // lastUpdate = millis();
      // pos += increment;
      // servo.write(pos);
      // Serial.println(pos);
      // if ((pos >= 180) || (pos <= 0)) // end of sweep
      // {
      //   // reverse direction..................................................................................000
      //   increment = -increment;
      // }
    }
  }

}

class DistanceSensor
{

}



void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
