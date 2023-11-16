/*
Fifth practice by:
GONZALEZ VELASCO OSCAR EDUARDO
PEREZ YANEZ MIGUEL ANGEL
*/

//Predetermined values
const float preDistance = 10.0f;
const float preRotation = 90.0f;
//Constants
#define MOTION Scroll(1000.0f)
#define FORWARD Scroll(preDistance)
#define BACK Scroll(-preDistance)
#define TURNLEFT RotateRobot(preRotation)
#define TURNRIGHT RotateRobot(-preRotation)
#define STOP Scroll(0.0f)
#define MSG Serial.println("Hola mundo!")

//Mathematic values
const float wheelRadius = 4.3f; // [cm]
const float distanceAxis = 8.9f; // [cm]
//Pin setup
const byte leftMotor = 5;  //~ (Goes to 2)
const byte rightMotor = 6; //~ (Goes to 15)
const byte vel1 = 10;      //~ (Goes to 1)
const byte vel2 = 11;      //~ (Goes to 9)
//Sensors
const byte leftContactSens = 13; 
const byte rightContactSens = 8; 
const byte currentBridge = 12;
const byte leftInfraSens = 19;   
const byte rightInfraSens = 9;
//Encoder
const byte rightEncoderA = 2;   //Interruption 1
const byte leftEncoderA = 3;    //Interruption 2
const byte leftEncoderB = 4;
const byte rightEncoderB = 7;
//LDR sensor
const byte LDR1 = 14; // Amarillo (A0)
const byte LDR2 = 15; // Blanco   (A1)
const byte LDR3 = 16; // Naranja  (A2)
const byte LDR4 = 17; // Verde    (A3)
const byte LDR5 = 18; // Morado   (A4)
//Interruptions
volatile long rightCounter;
volatile long leftCounter;
//Strings
String commandStr;
String command1;
String command2;
String command3;

void setup() 
{ 
  Serial.begin(9600);  
  /*===PIN SETUP===*/
  //Motors
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  pinMode(vel1, OUTPUT);
  pinMode(vel2, OUTPUT);
  //Sensors
  pinMode(currentBridge, OUTPUT);
  pinMode(leftInfraSens, INPUT); 
  pinMode(rightInfraSens, INPUT);
  pinMode(leftContactSens, INPUT_PULLUP);
  pinMode(rightContactSens, INPUT_PULLUP);
  //Encoders
  pinMode(leftEncoderA, INPUT);
  pinMode(rightEncoderA, INPUT);
  pinMode(leftEncoderB, INPUT);
  pinMode(rightEncoderB, INPUT);
  //LDR sensors
  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(LDR3, INPUT);
  pinMode(LDR4, INPUT);
  pinMode(LDR5, INPUT);
  /*===INITIAL VALUES===*/
  //Turns off motors
  digitalWrite(vel1, LOW);
  digitalWrite(vel2, LOW);
  digitalWrite(leftMotor, LOW);
  digitalWrite(rightMotor, LOW);  
  //Activates logic
  digitalWrite(currentBridge, HIGH);  
  //Encoder setup
  attachInterrupt(digitalPinToInterrupt(leftEncoderA), LeftEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), RightEncoderInterrupt, RISING);
}

void loop()
{
  //TestArrayLDR();
  //MaxLightValue();
  //SerialTestMotors();
  //ObstacleAvoidance();

  LeftControl(1200);
    
  /*/ 
  Scroll(0.0f); RotateRobot(90.0f);
  while(true) {} 
  // */
}

//Basic setup                 //Correct for left and right use
long previousTime = 0;
float previousError = 0;
float Iedt = 0;

bool prot = true;

void LeftControl(int target)
{
  //Constants
  float tolerance = 1.0f;
  
  //Control constants
  float Kp = 1.0f;
  float Ki = -0.01f;
  float Kd = -0.024f;

  //Time difference
  long currentPosition = leftCounter;
  long currentTime = micros();
  float dt = ((float)(currentTime - previousTime))/(1.0e6);
  previousTime = currentTime;  
  
  short error = currentPosition - target;
  
  //Derivative & Integral
  float Dedt = (error - previousError)/(dt);
  Iedt = Iedt + (error * dt);

  //PID Control Signal
  float uFunction = (Kp * error) + (Kd * Dedt) + (Ki * Iedt);

  int limitVelocity = 60;
  if(uFunction > limitVelocity)
    uFunction = limitVelocity;
  else if(uFunction < -limitVelocity)
    uFunction = -limitVelocity;

  /*
  Serial.print("u(t)= ");
  Serial.println(uFunction);*/

  if(abs(error) > 30)
    MotorMovement("L", uFunction);
  else
    MotorMovement("LOFF", 0);

  /*
  Serial.print("Error: ");
  Serial.println(error);*/
  
  Serial.print("Target: ");
  Serial.print(target);
  Serial.print(", ");
  Serial.print("Position: ");
  Serial.println(leftCounter);  
}

void RightControl(int target) //Check & Correct
{
  //Constants
  float tolerance = 1.0f;
  
  //Control constants
  float Kp = 1.0f;
  float Ki = -0.01f;
  float Kd = -0.024f;

  //Time difference
  long currentPosition = rightCounter;
  long currentTime = micros();
  float dt = ((float)(currentTime - previousTime))/(1.0e6);
  previousTime = currentTime;  
  
  short error = currentPosition - target;
  
  //Derivative & Integral
  float Dedt = (error - previousError)/(dt);
  Iedt = Iedt + (error * dt);

  //PID Control Signal
  float uFunction = (Kp * error) + (Kd * Dedt) + (Ki * Iedt);

  int limitVelocity = 60;
  if(uFunction > limitVelocity)
    uFunction = limitVelocity;
  else if(uFunction < -limitVelocity)
    uFunction = -limitVelocity;

  /*
  Serial.print("u(t)= ");
  Serial.println(uFunction);*/

  if(abs(error) > 30)
    MotorMovement("R", uFunction);
  else
    MotorMovement("ROFF", 0);

  /*
  Serial.print("Error: ");
  Serial.println(error);*/
  
  Serial.print("Target: ");
  Serial.print(target);
  Serial.print(", ");
  Serial.print("Position: ");
  Serial.println(leftCounter);  
}

void MotorMovement(String command, int speedPWM) 
{
  //Allows motor movement, SpeedPWM: Range[-127, 127]
  Serial.print("Command: ");
  Serial.print(command);
  Serial.print(", ");
  Serial.println(speedPWM);

  if(command == "L" || command == "R") //Can be tweaked yet
  {
    if(-10 < speedPWM && speedPWM < 10)
    {
      if(command == "L") 
        MotorMovement("LOFF", 0);
      else if(command == "R")
        MotorMovement("ROFF", 0);
      return;
    }  
  }
  
  speedPWM = Map(speedPWM);

  if(command == "L")
  {
    digitalWrite(vel1, HIGH);
    analogWrite(leftMotor, speedPWM);  
  }
  else if(command == "R")
  {
    digitalWrite(vel2, HIGH);
    analogWrite(rightMotor, speedPWM);
  }
  else if(command == "LOFF")
  {
    digitalWrite(leftMotor, LOW);
    digitalWrite(vel1, LOW); 
  }
  else if(command == "ROFF")
  {
    digitalWrite(rightMotor, LOW);
    digitalWrite(vel2, LOW); 
  }
  else if(command == "OFF")
  {
    digitalWrite(leftMotor, LOW);
    digitalWrite(vel1, LOW);
    digitalWrite(rightMotor, LOW);
    digitalWrite(vel2, LOW); 
  }
  else
    Serial.println("Unknown command!");
}

String ReadCommands()
{ 
  while (Serial.available() == 0) {} 
  commandStr = Serial.readString();
  commandStr.trim();  

  command1 = "";
  command2 = "";

  int wordCount = 0;
  for(int i = 0; i < commandStr.length(); i++)
  {
    if(commandStr[i] == ' ')      
      wordCount++;
    else if(wordCount == 0)
      command1 += commandStr[i];
    else if(wordCount == 1)
      command2 += commandStr[i];
  }
  return command1, command2;  
}

bool InfraredSensor(char sensor) 
{
  //Reads an infrared sensor and returns a boolean value if exceeds sensitivity
  float infraValue, sensitivity = 400.0f;
  
  if(sensor == 'L')
    infraValue = analogRead(leftInfraSens);
  else if(sensor == 'R')
    infraValue = analogRead(rightInfraSens);

  if(infraValue > sensitivity)
    return 0;
  else 
    return 1;
}

byte MaxLightValue()
{
  //Detects LDR with higher value and returns an index
  const byte sensorQuantity = 5;
  byte maxLightIndex = 0;
  unsigned int LDRvalue[sensorQuantity];
  
  for(byte i = 0; i < sensorQuantity; i++)
    LDRvalue[i] = analogRead(i + 14);
    
  for(byte i = 1; i < sensorQuantity; i++)
    if(LDRvalue[maxLightIndex] < LDRvalue[i])
      maxLightIndex = i;

  //
  Serial.print("Max value: ");
  Serial.println(maxLightIndex);
  // */
  return maxLightIndex; 
}

bool ContactSensor(char sensor)
{
  //Reads a contact sensor and returns a boolean value
  bool registeredValue;

  if(sensor == 'L')
    registeredValue = digitalRead(leftContactSens);
  else if(sensor == 'R')
    registeredValue = digitalRead(rightContactSens);
  return registeredValue; 
}

void RotateRobot(float angle)
{ 
  //Rotates robot a given angle (Not very precise)
  const float baseRotation = 90.0f;
  const float baseTime = 900.0f;
  const int deviation = 50;
  int leftVel = 40;
  int rightVel = 40;
  
  float redefine = abs(angle) / baseRotation;

  if(angle < 0)
  {
    leftVel *= -1;
    rightVel *= -1;
  }

  MotorMovement("R", rightVel);
  MotorMovement("L", leftVel);
  delay(int(baseTime * redefine) + deviation);
  MotorMovement("OFF", 0);
}

void Scroll(float distance)
{
  //Allows linear movement in a given distance in [cm] (Not very precise)
  const float baseDistance = 10.0f;
  const float baseTime = 700.0f;
  const int deviation = 50;
  const int leftVel = 45;
  const int rightVel = 38; 
  
  float redefine = abs(distance) / baseDistance;

  if(distance > 999)
  {
    MotorMovement("L", -leftVel);
    MotorMovement("R", rightVel);
  }
  else if(distance > 0)
  {
    MotorMovement("L", -leftVel);
    MotorMovement("R", rightVel);
    delay(int(baseTime * redefine) + deviation);
    MotorMovement("OFF", 0);
  }
  else if(distance < 0) 
  {
    MotorMovement("L", leftVel);
    MotorMovement("R", -rightVel);
    delay(int(baseTime * redefine) + deviation);
    MotorMovement("OFF", 0);
  }
  else if(distance == 0)
    MotorMovement("OFF", 0);
}

void MoveRobot(float angle, float distance)
{
  //First: Move to angle, then move distance
  RotateRobot(angle);
  Scroll(distance);
}

void ObstacleAvoidance() //Obstacle avoidance algorithm 
{
  /*  COMMANDS
  ContactSensor('L', 'R');
  FORWARD
  BACK
  TURNLEFT
  TURNRIGHT
  STOP
  */

  int state = 0;

  state = 3 - (ContactSensor('L') * 2 + ContactSensor('R') * 1);

  /*
  if(state == 0)
    state = 3 - (InfraredSensor('L') * 2 + InfraredSensor('R') * 1);*/

  switch(state)
  {
    case 0: //No obstacle in front
      MOTION;
      break;
    case 1: //Obstacle to the right
      STOP;
      BACK;
      TURNLEFT;
      break;
    case 2: //Obstacle to the left
      STOP;
      BACK;
      TURNRIGHT;
      break;
    case 3: //Obstacle in front of the robot
      STOP;
      BACK;
      TURNLEFT;
      TURNLEFT;
      FORWARD;
      TURNRIGHT;
      break;
 }
}

void LeftEncoderInterrupt()
{
  if(digitalRead(leftEncoderA) == HIGH)
    if(digitalRead(leftEncoderB) == LOW)
      leftCounter--;
    else
      leftCounter++;
  else
    if(digitalRead(leftEncoderB) == LOW)
      leftCounter++;
    else
      leftCounter--;
}

void RightEncoderInterrupt()
{
  if(digitalRead(rightEncoderA) == HIGH)
    if(digitalRead(rightEncoderB) == LOW)
      rightCounter++;
    else
      rightCounter--;
  else
    if(digitalRead(rightEncoderB) == LOW)
      rightCounter--;
    else
      rightCounter++;
}

void TestContact()
{
  Serial.print("Left: ");   Serial.println(ContactSensor('L'));
  Serial.print("Right: ");  Serial.println(ContactSensor('R'));
  delay(1000);
}

void TestInfrared()
{
  Serial.print("Lectura Izq: "); 
  Serial.print(analogRead(leftInfraSens)); 
  //Serial.println(InfraredSensor('L'));

  Serial.print(", Lectura Derecha: "); 
  Serial.print(analogRead(rightInfraSens)); 
  //Serial.println(InfraredSensor('R'));

  delay(600);
}

void TestEncoders()
{
  Serial.print("Right: ");
  Serial.print(rightCounter);
  Serial.print(", Left: ");
  Serial.println(leftCounter);
  delay(100);
}

void TestArrayLDR()
{
  const byte sensorQuantity = 5;
  unsigned int LDRvalue[sensorQuantity];
  
  for(byte i = 0; i < sensorQuantity; i++)
    LDRvalue[i] = analogRead(i + 14);
  
  Serial.print("Valores: ");
  for(byte i = 0; i < sensorQuantity; i++)
  {
    Serial.print(LDRvalue[i]);
    Serial.print(", ");
  }
  Serial.println(" ");
  delay(1000);
}

void SerialTestMotors()
{
  command1, command2 = ReadCommands();
  MotorMovement(command1, command2.toInt());
}

short Map(short value) 
{
  //Input: Range[-127, 127], Output: Range[0, 255]
  value += 127;
  return value;
}

short InverseMap(short value)
{
  //Input: Range[0, 255], Output: Range[-127, 127]
  value -= 127;
  return value;
}

short AngleCorrection(short angle) //Check & correct
{
  bool constant = 1;
  
  if(angle < 0)
    constant *= -1;  
  while(angle > 180)
    angle -= 180;
  angle *= constant;
  return angle;
}

volatile long LeftTickValue()
{
  return leftCounter;
}

volatile long RightTickValue()
{
  return rightCounter;
}
