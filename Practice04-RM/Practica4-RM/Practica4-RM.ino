/*
Fourth practice by:
GONZALEZ VELASCO OSCAR EDUARDO
PEREZ YANEZ MIGUEL ANGEL
*/

//Predetermined values
const float predeterminedDistance = 10.0f;
const float predeterminedRotation = 45.0f;

//Constants
#define MOTION Scroll(1000.0f)
#define FORWARD Scroll(10.0f)
#define BACK Scroll(-10.0f)
#define TURNLEFT RotateRobot(45.0f)
#define TURNRIGHT RotateRobot(-45.0f)
#define STOP Scroll(0.0f)
#define MSG Serial.println("Hola mundo!")

//Pin setup
const int LeftMotor = 6;  //~ (Goes to 2)
const int RightMotor = 5; //~ (Goes to 15)
const int Vel1 = 11;      //~ (Goes to 1)
const int Vel2 = 10;      //~ (Goes to 9)

const int leftContactSens = 2; // 
const int rightContactSens = 3; //
const int currentBridge = 12;

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

  //Set to 0 all values
  digitalWrite(Vel1, LOW);
  digitalWrite(Vel2, LOW);
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);  

  digitalWrite(currentBridge, HIGH);  
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

int map(int value)
{
  value += 127;
  return value;
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

//Motor A1 -> Derecho, Motor A2 -> Izquierdo
void MotorCommand(String command1, String command2, String command3)
{
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

void Action(String command1, String command2, String command3)
{
  if(command1 == "SENSOR")
    Serial.println(ContactSensor(command2.toInt()));
  else if(command1 == "MOVE")
    MoveRobot(command2.toFloat(), command3.toFloat()); 
  else if(command1 == "ROTATION")
    RotateRobot(command2.toInt());
  else
    Serial.println("Unknown!");
}

void RotateRobot(float angle)
{ 
  //This parameters show a 90 degree rotation
  const float baseRotation = 90.0f;
  const float baseTime = 845.0f;
  const int deviation = 50;
  
  float redefine = abs(angle) / baseRotation;

  if(angle > 0)
  {
    MotorCommand("A1", "SPEED", "40");
    MotorCommand("A2", "SPEED", "40");
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else if(angle < 0)
  {
    MotorCommand("A1", "SPEED", "-40");
    MotorCommand("A2", "SPEED", "-40");
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
  const float baseTime = 780.0f;
  const int deviation = 50;
  
  float redefine = abs(distance) / baseDistance;

  if(distance > 999)
  {
    MotorCommand("A1", "SPEED", "45");
    MotorCommand("A2", "SPEED", "-45");
  }
  else if(distance > 0)
  {
    MotorCommand("A1", "SPEED", "45");
    MotorCommand("A2", "SPEED", "-45");
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else if(distance < 0) 
  {
    MotorCommand("A1", "SPEED", "-45");
    MotorCommand("A2", "SPEED", "45");
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else if(distance == 0)
  {
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else{ }
}

void MoveRobot(float angle, float distance) //Distance in [cm], angle in degrees [°]
{
  //First: Move to angle, then move distance
  RotateRobot(angle);
  Scroll(distance);
}

void ObstacleAvoidance() //Obstacle avoidance algorithm (State machines pendient)
{
  /*  COMMANDS
  ContactSensor('L', 'R');
  FORWARD
  BACK
  TURNLEFT
  TURNRIGHT
  STOP
  */
  
  if(ContactSensor('R') == LOW) //0 = SI encontró obstáculo 
  {
    STOP;
    if(ContactSensor('L') == LOW) //SI encontró obstáculo
    {
      BACK;
      TURNLEFT;
      TURNLEFT;
      FORWARD;
      TURNRIGHT;
      TURNRIGHT;
    }
    else //NO encontró obstáculo
    {
      BACK;
      TURNLEFT;
      FORWARD;
      TURNRIGHT;
    }
  }
  else if(ContactSensor('L') == LOW)
  {
    STOP;
    BACK;
    TURNRIGHT;
    FORWARD;
    TURNLEFT;
  }
  else { MOTION; }
}

int x = 0;
void loop() 
{
  delay(1000); //A brief interval before starting

  if(x == 0)
  {
    MOTION;
    x++;
    Serial.println("Sólo debe aparecer una vez");
  }
    
  ObstacleAvoidance();

  /*Scroll(20.0f);
  RotateRobot(90.0f);
  while(true) {} /*/

  /*
  Serial.print("Left: ");
  Serial.println(ContactSensor('L'));
  Serial.print("Right: ");
  Serial.println(ContactSensor('R'));
  */
}
