char input;
float pot1;
float pot2;

void setup() 
{
  Serial.begin(9600);  
  pinMode(3, OUTPUT);
}

void loop() 
{
  SendData();
  ReceiveData();
}

void ReceiveData()
{
  if(Serial.available() > 0)
  {
    input = Serial.read();

    if(input == 'a')
      digitalWrite(3, HIGH);
    else if(input == 'b')
      digitalWrite(3, LOW);
  }
}

void SendData()
{
  pot1 = analogRead(A0);
  pot2 = analogRead(A1);

  Serial.print(pot1);
  Serial.print(",");
  Serial.println(pot2);
  delay(10);
}
