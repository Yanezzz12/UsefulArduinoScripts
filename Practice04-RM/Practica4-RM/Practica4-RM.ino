/*
Fourth practice by:
GONZALEZ VELASCO OSCAR EDUARDO
PEREZ YANEZ MIGUEL ANGEL
*/

//Predetermined values
const float preDistance = 10.0f;
const float preRotation = 50.0f;

//Constants
#define MOTION Scroll(1000.0f)
#define FORWARD Scroll(10.0f)
#define BACK Scroll(-10.0f)
#define TURNLEFT RotateRobot(preRotation)
#define TURNRIGHT RotateRobot(-preRotation)
#define STOP Scroll(0.0f)
#define MSG Serial.println("Hola mundo!")

//Pin setup
const int LeftMotor = 6;  //~ (Goes to 2)
const int RightMotor = 5; //~ (Goes to 15)
const int Vel1 = 11;      //~ (Goes to 1)
const int Vel2 = 10;      //~ (Goes to 9)

const int leftContactSens = 2; 
const int rightContactSens = 3; 
const int currentBridge = 12;
const int leftInfraSens = A0;
const int rightInfraSens = A5;

void setup() 
{ 
  Serial.begin(9600);  

  //Initializinng variables
  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);
  pinMode(Vel1, OUTPUT);
  pinMode(Vel2, OUTPUT);

  pinMode(currentBridge, OUTPUT);
  pinMode(leftContactSens, INPUT_PULLUP);
  pinMode(rightContactSens, INPUT_PULLUP);
  pinMode(leftInfraSens, INPUT); 
  pinMode(rightInfraSens, INPUT);

  //Set to 0 all values
  digitalWrite(Vel1, LOW);
  digitalWrite(Vel2, LOW);
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);  

  digitalWrite(currentBridge, HIGH);  

  /*
  attachInterrupt(digitalPinToInterrupt(leftContactSens), InterruptProcess, LOW);
  attachInterrupt(digitalPinToInterrupt(rightContactSens), InterruptProcess, LOW);
  */
}

bool InfraredSensor(char sensor)
{
  float infraValue;
  float sensibilidad = 400.0f;
  
  if(sensor == 'L')
    infraValue = analogRead(leftInfraSens);
  else if(sensor == 'R')
    infraValue = analogRead(rightInfraSens);

  if(infraValue > sensibilidad)
    return 0;
  else 
    return 1;
}

int ContactSensor(char sensor)
{
  int registeredValue;

  if(sensor == 'L')
    registeredValue = digitalRead(leftContactSens);
  else if(sensor == 'R')
    registeredValue = digitalRead(rightContactSens);
  return registeredValue; 
}

void RotateRobot(float angle)
{ 
  //This parameters show a 90 degree rotation
  const float baseRotation = 90.0f;
  const float baseTime = 800.0f;
  const int deviation = 50;
  const String leftVel = "40";
  const String rightVel = "40";
  
  float redefine = abs(angle) / baseRotation;

  if(angle > 0)
  {
    MotorCommand("A1", "SPEED", rightVel);
    MotorCommand("A2", "SPEED", leftVel);
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else if(angle < 0)
  {
    MotorCommand("A1", "SPEED", "-" + rightVel);
    MotorCommand("A2", "SPEED", "-" + leftVel);
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else{}
}

void Scroll(float distance)
{
  //This parameters shows a 10 [cm] scroll
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
    case 0:
      MOTION;
      break;
    case 1:
      STOP;
      BACK;
      TURNLEFT;
      break;
    case 2:
      STOP;
      BACK;
      TURNRIGHT;
      break;
    case 3:
      STOP;
      BACK;
      TURNLEFT;
      TURNLEFT;
      FORWARD;
      TURNRIGHT;
      break;
 }
}

void loop() 
{
  ObstacleAvoidance();
  
  /*
  Scroll(20.0f);  RotateRobot(90.0f);
  while(true) {} 
  // */

  /*
  Serial.print("Lectura Izq: "); Serial.println(InfraredSensor('L'));
  Serial.print("Lectura Dch: "); Serial.println(InfraredSensor('R'));
  delay(1000);
  // */

  /*
  Serial.print("Left: ");   Serial.println(ContactSensor('L'));
  Serial.print("Right: ");  Serial.println(ContactSensor('R'));
  delay(1000);
  // */
}

String ReadCommands()
{ 
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

void MotorCommand(String command1, String command2, String command3)
{
  //Motor A1 -> Derecho, Motor A2 -> Izquierdo
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

void MoveRobot(float angle, float distance)
{
  //First: Move to angle, then move distance
  RotateRobot(angle);
  Scroll(distance);
}

int map(int value)
{
  value += 127;
  return value;
}
