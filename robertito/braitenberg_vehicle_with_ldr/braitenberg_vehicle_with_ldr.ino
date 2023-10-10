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


}

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


}

class Robot
{
  //Define properties
  ///Motors
  Motor leftMotor;
  Motor rightMotor;
  const int speed = 240;
  ///LDRs
  Ldr leftLdr;
  Ldr rightLdr;
  const int maxLighting = 500;
  ///Distance sensor
  DistanceSensor distanceSensor;
  const int maxDistance = 10;
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
      int distance = this->distanceSensor.calculateDistance();
      if(distance<maxDistance){
        this->leftMotor.backward(speed);
        this->leftMotor.backward(speed);
      }
    }

    //update Ldrs
    if((millis() - this->ldrLastUpdate) > this->ldrUpdateInterval)  // time to update ldrs
    {
      this->ldrLastUpdate = millis();
      int leftLight = this->leftLdr.calculateLight();
      int rightLight = this->rightLdr.calculateLight();
      //Left motor update
      if(leftLight > maxLight){
        this->leftMotor.forward(speed);
      }
      else{
        this->leftMotor.stop(speed);
      }
      //Right motor update
      if(rightLight > maxLight){
        this->rightMotor.forward(speed);
      }
      else{
        this->rightMotor.stop(speed);
      }
    }
  }

}

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



}

Robot robot;


void setup() {
  //Initialize robot and his components

  //Components

  ///Motors
  ////Left motor
  Motor leftMotor(3, 5, 4);
  leftMotor.setup();
  ///Right motor
  Motor rightMotor(9, 8, 7);
  rightMotor.setup();

  ///Ldrs
  ////Left ldr
  Ldr leftLdr(1);
  ////Right ldr
  Ldr rightLdr(2);

  ///Distance sensor
  DistanceSensor distanceSensor(11,12);
  distanceSensor.setup();

  //Robot
  int distanceSensorUpdateInterval = 15;
  int ldrUpdateInterval = 10;
  robot(leftMotor,rightMotor,leftLdr,rightLdr,distanceSensor,distanceSensorUpdateInterval,ldrUpdateInterval);


}

void loop() {
  robot.Update();
}
