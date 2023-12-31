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
/* WheelDiameter = 4.3f cm
 * WheelPerimeter = 13.51f cm
 * DistanceBetweenWheels = 8.9f cm
 */
const float wheelPerimeter = 13.51f; //cm
const byte ticksPerRevolution = 300;
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
const byte LDR1 = 14; // Yellow   (A0)
const byte LDR2 = 15; // White    (A1)
const byte LDR3 = 16; // Orange   (A2)
const byte LDR4 = 17; // Green    (A3)
const byte LDR5 = 18; // Purple   (A4)
//Interruptions
volatile long rightCounter;
volatile long leftCounter;
//PID Control
long previousTime[2] = {0, 0};
float previousError[2] = {0, 0};
float Iedt[2] = {0, 0}; 
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

  //ScrollPID(5.0f);
  //RightPID(100);

  //MotorPID(300, 'R');
  //MotorMovement("R", 40);

  //TestArrayLDR();
  //TestEncoders();

  /*/ 
  Scroll(0.0f); RotateRobot(90.0f);
  while(true) {} 
  // */
}

void MotorPID(long target, char symbol)
{
  //Control boundaries
  const float tolerance = 6.0f;
  const float limitVelocity = 60.0f;
  const float minVelocity = 20.0f;
  
  //Control constants
  const float Kp = 5.9f;    // Kp = 6.0f
  const float Ki = 0.01f;   // Ki = 
  const float Kd = 0.7f;    // Kd = 0.003f

  //Variable declaration 
  const short motorQuantity = 2;
  long currentPosition[motorQuantity];
  long currentTime[motorQuantity];
  float dt[motorQuantity];
  short error[motorQuantity];
  float Dedt[motorQuantity];
  float uFunction[motorQuantity];
  long destiny[motorQuantity];

  short motor;
  if(symbol == 'L')
  {
    motor = 0; 
    currentPosition[motor] = leftCounter; //Time difference
  }
  else if(symbol == 'R')
  {
    motor = 1;
    currentPosition[motor] = rightCounter; //Time difference
  }

  destiny[motor] = target;
  currentTime[motor] = micros();
  dt[motor] = ((float)(currentTime[motor] - previousTime[motor]))/(1.0e6);
  previousTime[motor] = currentTime[motor];
  error[motor] = currentPosition[motor] - destiny[motor]; //Check target too
  
  //Derivative & Integral
  Dedt[motor] = (error[motor] - previousError[motor])/(dt[motor]);
  Iedt[motor] = Iedt[motor] + (error[motor] * dt[motor]); //Convert into array

  //PID Control Signal
  uFunction[motor] = (Kp * error[motor]) + (Kd * Dedt[motor]) + (Ki * Iedt[motor]);
  //uFunction = ErrorToPWM(target, uFunction);

  //Signal processing & Plant
  if(uFunction[motor] > limitVelocity)
    uFunction[motor] = limitVelocity;
  else if(uFunction[motor] < -limitVelocity)
    uFunction[motor] = -limitVelocity;
  else if((uFunction[motor] < minVelocity) && (uFunction[motor] > 0))
    uFunction[motor] = minVelocity;
  else if((uFunction[motor] < 0) && (uFunction[motor] > -minVelocity)) 
    uFunction[motor] = -minVelocity;
  
  previousError[motor] = error[motor];

  if(abs(error[motor]) > tolerance)
  {
    if(symbol == 'L')
      MotorMovement("L", uFunction[motor]);
    else if(symbol == 'R')
      MotorMovement("R", uFunction[motor]);
  }
  else
  {
    if(symbol == 'L')
      MotorMovement("LOFF", 0);
    else if(symbol == 'R')
      MotorMovement("ROFF", 0);
  }
  //END of plant

  //Graphing tool in Serial Plotter
  Serial.print(abs(target));          Serial.print(", ");
  Serial.print(abs(leftCounter));   Serial.print(", "); 
  Serial.println(abs(rightCounter));    
}

void ScrollPID(float distance)
{
  long ticks = DistanceToTicks(distance);
  MotorPID(ticks, 'L');
  MotorPID(ticks, 'R');
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
    analogWrite(rightMotor, -speedPWM);
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
  Serial.print("Left: ");
  Serial.print(leftCounter);
  Serial.print(", Right: ");
  Serial.println(rightCounter);
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

short AngleCorrection(short angle)
{  
  while(angle > 180)
    angle -= 360;
  while(angle < 180)
    angle += 360;
  return angle;
}

float ErrorToPWM(long maxError, float currentError)
{
  /* This function transforms an e(t) to a F(s) signal able to get in the plant (Motor)
   * The mathematic model is linear, described by m = (maxPWMValue - minPWMValue) / (maxError - minError)
   * m = (85 - 19) / (maxError - 0) => m = 66/target => F(s) = [66*e(t)]/target + minPWMValue
  */
  currentError = (66.0f * currentError)/maxError + 20.0f;
  return currentError;
}

long DistanceToTicks(float distance)
{
  long ticks = 22.206f * distance;
  return ticks;
}

volatile long LeftTickValue()
{
  return leftCounter;
}

volatile long RightTickValue()
{
  return rightCounter;
}
