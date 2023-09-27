/*
Third practice by:
PEREZ YANEZ MIGUEL ANGEL
GONZALEZ VELASCO OSCAR EDUARDO
*/

//Pin setup
int LeftMotor = 6;  //~ (Goes to 2)
int RightMotor = 5; //~ (Goes to 15)
int Vel1 = 11;      //~ (Goes to 1)
int Vel2 = 10;      //~ (Goes to 9)

//Variables
int speedPWM = 0;

String commandStr;
String command1;
String command2;
String command3;

void setup() 
{
  Serial.begin(9600);

  //Set to 0 all values
  digitalWrite(Vel1, LOW);
  digitalWrite(Vel2, LOW);
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);
  
  //Initializing 
  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);
  pinMode(Vel1, OUTPUT);
  pinMode(Vel2, OUTPUT);

  
}

String ReadCommands()
{ 
  while (Serial.available() == 0) {} 
  commandStr = Serial.readString();
  commandStr.trim();  

  command1 = "";
  command2 = "";
  command3 = "";

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

int map(int value)
{
  value += 127;
  return value;
}

int baseVelocity = 40;
void loop() 
{
  command1, command2, command3 = ReadCommands();

  if(command1 == "A1")
  {
    if(command2 == "ON") 
    {
      if(command3 == "LEFT")
      {
        digitalWrite(LeftMotor, LOW);
        analogWrite(Vel1, baseVelocity);  
      }
      else if(command3 == "RIGHT")
      {
        digitalWrite(LeftMotor, HIGH);
        analogWrite(Vel1, baseVelocity);
      }
    }
    else if(command2 == "SPEED")
    {
      speedPWM = command3.toInt();
      speedPWM = map(speedPWM);
      
      Serial.print("Velocity set to: ");
      Serial.println(speedPWM);

      analogWrite(LeftMotor, speedPWM);
      digitalWrite(Vel1, HIGH);
    }
    else if(command2 == "OFF") 
    {
      digitalWrite(LeftMotor, LOW);
      analogWrite(Vel1, 0);  
    }
  }
  else if(command1 == "A2")
  {
    if(command2 == "ON") 
    {
      if(command3 == "LEFT")
      {
        digitalWrite(RightMotor, LOW);
        analogWrite(Vel2, baseVelocity);  
      }
      else if(command3 == "RIGHT")
      {
        digitalWrite(RightMotor, HIGH);
        analogWrite(Vel2, baseVelocity);
      }
    }
    else if(command2 == "SPEED")
    {
      speedPWM = command3.toInt();
      speedPWM = map(speedPWM);
      
      Serial.print("Velocity set to: ");
      Serial.println(speedPWM);

      analogWrite(RightMotor, speedPWM);
      digitalWrite(Vel2, HIGH);
    }
    else if(command2 == "OFF") 
    {
      digitalWrite(RightMotor, LOW);
      analogWrite(Vel2, 0);  
    }
  }
  else
  {
    Serial.println("Command not found");  
  }
}
