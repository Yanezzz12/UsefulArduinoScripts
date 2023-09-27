/*
Script by Miguel Yanez
Reads a string in the serial monitor and separates
every parameter.
*/

//Variables
String commandStr;
String command1;
String command2;
String command3;
  
void setup() 
{
  Serial.begin(9600);
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




void loop() 
{ 
  command1, command2, command3 = ReadCommands();
  
  Serial.println(command1);
  Serial.println("Space");
  Serial.println(command2);
  Serial.println("Space");
  Serial.println(command3);
}
