/*
Fourth practice by:
GONZALEZ VELASCO OSCAR EDUARDO
PEREZ YANEZ MIGUEL ANGEL
*/

//Constants
#define ADELANTE move_robot(AVANCE, 0.0f) //Definir bien
#define ATRAS move_robot(-AVANCE, 0.0)
#define GIRO_IZQ move_robot(0.0, GIRO)
#define GIRO_DER move_robot(0.0, -GIRO)
#define ALTO move_robot(0.0,0.0)

//Pin setup
const int LeftMotor = 6;  //~ (Goes to 2)
const int RightMotor = 5; //~ (Goes to 15)
const int Vel1 = 11;      //~ (Goes to 1)
const int Vel2 = 10;      //~ (Goes to 9)

const int contactSensorA; // = ??
const int contactSensorB; // = ??
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

  //Set to 0 all values
  digitalWrite(Vel1, LOW);
  digitalWrite(Vel2, LOW);
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);  

  digitalWrite(currentBridge, HIGH);  
}

int ContactSensor(int numSensor)
{
  int registeredValue;

  if(numSensor == 1)
    registeredValue = digitalRead(contactSensorA);
  else if(numSensor == 2)
    registeredValue = digitalRead(contactSensorB);
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

void MotorCommand(String command1, String command2, String command3)
{
  int speedPWM = 0;
  int baseVelocity = 40;
  
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
    Serial.println("Velocity set to: " + String(speedPWM));
    
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
  else if(command1 == "ROTATION")
    RotateRobot(command2.toInt());
  else
    Serial.println("Unknown!");
}

void RotateRobot(float angle)
{ 
  //This parameters show a 90 degree rotation
  float baseRotation = 90.0f;
  float baseTime = 875.0f;
  int deviation = 50;
  
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

void Scroll(float distance) //Correct this parameters
{
  //This parameters shows a 10 [cm] scroll
  float baseDistance = 10.0f;
  float baseTime = 875.0f;
  int deviation = 50;
  
  float redefine = abs(distance) / baseDistance;

  if(distance > 0)
  {
    MotorCommand("A1", "SPEED", "40");
    MotorCommand("A2", "SPEED", "-40");
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else if(distance < 0) 
  {
    MotorCommand("A1", "SPEED", "-40");
    MotorCommand("A2", "SPEED", "40");
    delay(int(baseTime * redefine) + deviation);
    MotorCommand("A1", "OFF", "");
    MotorCommand("A2","OFF", "");
  }
  else{}
}

void MoveRobot(float angle, float distance) //Distance in [cm], angle in degrees [°]
{
  //First: Move to angle, then move distance
  RotateRobot(angle);
  Scroll(distance);
}

void loop() 
{
  /*
  MotorCommand(command1, command2, command3);
  */

  delay(1000); //A brief interval before starting

  //command1, command2, command3 = ReadCommands();
  //Action(command1, command2, command3);

  MoveRobot(10, 90);

  while(true) {}



}
