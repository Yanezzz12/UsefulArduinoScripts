/*
Fifth practice by:
GONZALEZ VELASCO OSCAR EDUARDO
PEREZ YANEZ MIGUEL ANGEL
*/

//Predetermined values
const float preDistance = 10.0f;
const float preRotation = 50.0f;
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
//Pin setup
const byte LeftMotor = 6;  //~ (Goes to 2)
const byte RightMotor = 5; //~ (Goes to 15)
const byte Vel1 = 11;      //~ (Goes to 1)
const byte Vel2 = 10;      //~ (Goes to 9)
//Sensors
const byte leftContactSens = 13; 
const byte rightContactSens = 8; 
const byte currentBridge = 12;
const byte leftInfraSens = 19;   
const byte rightInfraSens = 9;
//Encoder
const byte rightEncoderA = 2; //Interruption 1
const byte leftEncoderA = 3; //Interruption 2
const byte leftEncoderB = 4;
const byte rightEncoderB = 7;
//LDR sensor
const byte LDR1 = 14;
const byte LDR2 = 15;
const byte LDR3 = 16;
const byte LDR4 = 17;
const byte LDR5 = 18;

void setup() 
{ 
  Serial.begin(9600);  

  /*===PIN SETUP===*/
  //Motors
  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);
  pinMode(Vel1, OUTPUT);
  pinMode(Vel2, OUTPUT);

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
  digitalWrite(Vel1, LOW);
  digitalWrite(Vel2, LOW);
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);  

  //Activates logic
  digitalWrite(currentBridge, HIGH);  

  //Encoder setup
  attachInterrupt(digitalPinToInterrupt(leftEncoderA), LeftEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), RightEncoderInterrupt, RISING);
}

void loop()
{
  //ObstacleAvoidance();

  /*/
  Serial.print("Right: ");
  Serial.println(rightCounter);

  Serial.print("Left: ");
  Serial.println(leftCounter);
  delay(100);
  // */
  
  /*/ 
  Scroll(0.0f);  RotateRobot(90.0f);
  while(true) {} 
  // */

  /*/
  Serial.print("Lectura Izq: "); Serial.print(analogRead(leftInfraSens)); Serial.print(" "); //Serial.println(InfraredSensor('L'));
  Serial.print("Lectura Dch: "); Serial.println(analogRead(rightInfraSens)); //Serial.println(InfraredSensor('R'));
  delay(600);
  // */

  /*/
  Serial.print("Left: ");   Serial.println(ContactSensor('L'));
  Serial.print("Right: ");  Serial.println(ContactSensor('R'));
  delay(1000);
  // */
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
  byte sensorQuantity = 5;
  byte maxLightIndex = 0;
  unsigned int LDRvalue[sensorQuantity];
  
  for(byte i = 0; i < sensorQuantity + 1; i++)
    LDRvalue[i] = analogRead(i + 14);

  /*/ //Shows LDR values
  Serial.print("Arreglo de LDRs: ");
  for(byte i = 0; i < sensorQuantity + 1; i++)
    Serial.println(LDRvalue[i]);*/

  for(byte i = 1; i < sensorQuantity + 1; i++)
    if(LDRvalue[maxLightIndex] < LDRvalue[i])
      maxLightIndex = i;
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
  String leftVel = "40";
  String rightVel = "40";
  
  float redefine = abs(angle) / baseRotation;

  if(angle < 0)
  {
    leftVel = "-" + leftVel;
    rightVel = "-" + rightVel;
  }

  MotorCommand("A1", "SPEED", rightVel);
  MotorCommand("A2", "SPEED", leftVel);
  delay(int(baseTime * redefine) + deviation);
  MotorCommand("A1", "OFF", "");
  MotorCommand("A2","OFF", "");
}

void EncoderRotate(float angle)
{
  String leftVel = "40";
  String rightVel = "40";
  float baseRotation = 90.0f;
  float baseTicks = 170.0f;

  int leftCounter = LeftTickValue();
  int rightCounter = RightTickValue();

  baseTicks = baseTicks * (abs(angle) / baseRotation);
  Serial.print("Base Ticks: ");
  Serial.println(baseTicks);

  if(angle < 0)
  {
    leftVel = "-" + leftVel;
    rightVel = "-" + rightVel;
  }  

  if(abs(leftCounter) > int(baseTicks))
    MotorCommand("A2","OFF", "");
  else 
    MotorCommand("A2", "SPEED", leftVel);  

  if(abs(rightCounter) > int(baseTicks))
    MotorCommand("A1","OFF", "");
  else 
    MotorCommand("A1", "SPEED", rightVel);
}

void Scroll(float distance)
{
  //Allows linear movement in a given distance in [cm] (Not very precise)
  const float baseDistance = 10.0f;
  const float baseTime = 700.0f;
  const int deviation = 50;
  const String leftVel = "45";
  const String rightVel = "38"; //54
  
  float redefine = abs(distance) / baseDistance;

  if(distance > 999)
  {
    MotorCommand("A1", "SPEED", rightVel);
    MotorCommand("A2", "SPEED", "-" + leftVel);
  }
  else if(distance > 0)
  {
    MotorCommand("A1", "SPEED", leftVel);
    MotorCommand("A2", "SPEED", "-" + rightVel);
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else if(distance < 0) 
  {
    MotorCommand("A1", "SPEED", "-" + leftVel);
    MotorCommand("A2", "SPEED", rightVel);
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else if(distance == 0)
  {
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else { }
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

  if(state == 0)
    state = 3 - (InfraredSensor('L') * 2 + InfraredSensor('R') * 1);

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

volatile long leftCounter;
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

volatile long rightCounter;
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

String ReadCommands()
{ 
  //Reads a string from serial monitor and decomposes in arguments to be stored in command_n
  String commandStr;
  String command1 = "";
  String command2 = "";
  String command3 = "";
  
  while (Serial.available() == 0) {} 
  commandStr = Serial.readString();
  commandStr.trim();  

  int wordCount = 0;
  for(int i = 0; i < commandStr.length(); i++)
  {
    if(commandStr[i] == ' ')      
      wordCount++;
    else if(wordCount == 0)
      command1 += commandStr[i];
    else if(wordCount == 1)
      command2 += commandStr[i];
    else if(wordCount == 2)
      command3 += commandStr[i];
  }
  return command1, command2, command3;  
}

void MotorCommand(String command1, String command2, String command3) //Refactor needed
{
  //Allows motor control (A1 -> Right, A2 -> Left)
  const int baseVelocity = 40;
  int speedPWM = 0;
  
  if(command2 == "ON")
  { 
    if(command1 == "A1")
    {
      if(command3 == "LEFT" ||  command3 == "RIGHT")
        analogWrite(Vel1, baseVelocity);
        
      if(command3 == "LEFT")
        digitalWrite(LeftMotor, LOW);
      else if(command3 == "RIGHT")
        digitalWrite(LeftMotor, HIGH);
    }
    else if(command1 = "A2")
    {
      if(command3 == "LEFT" ||  command3 == "RIGHT")
        analogWrite(Vel2, baseVelocity);
      
      if(command3 == "LEFT")
        digitalWrite(RightMotor, LOW);
      else if(command3 == "RIGHT")
        digitalWrite(RightMotor, HIGH);
    }  
  }
  else if(command2 == "SPEED") 
  {
    speedPWM = command3.toInt();

    //Protects circuits
    if(-10 < speedPWM && speedPWM < 10)
    {
      MotorCommand(command1, "OFF", command3);
      return;
    }
    
    speedPWM = map(speedPWM);
    
    if(command1 == "A1")
    {
      digitalWrite(Vel1, HIGH);
      analogWrite(LeftMotor, speedPWM);    
    }
    else if(command1 = "A2")
    {
      digitalWrite(Vel2, HIGH);
      analogWrite(RightMotor, speedPWM);  
    }
  }
  else if(command2 == "OFF")
  {
    if(command1 == "A1") //Check if can be tweaked
    {
      digitalWrite(LeftMotor, LOW);
      digitalWrite(Vel1, LOW);  
    }
    else if(command1 == "A2")
    {
      digitalWrite(RightMotor, LOW);
      digitalWrite(Vel2, LOW); 
    }
  }
  else
    Serial.println("Unknown command!");
}

int map(int value) 
{
  //Input: Range[-127, 127], Output: Range[0, 255]
  value += 127;
  return value;
}

volatile long LeftTickValue()
{
  return leftCounter;
}

volatile long RightTickValue()
{
  return rightCounter;
}
