//Variables declaration
int delayTime;
const int frequency = 500; //Value in [Hz]

// Definir pines conectados a las bobinas del driver
#define IN1  2
#define IN2  3
#define IN3  4
#define IN4  5
 
// Secuencia de pasos a par máximo del motor. Realmente es una matriz que representa la tabla del unipolar que he mostrado antes
int paso [4][4] =
{
  {1, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 1},
  {1, 0, 0, 1}
};

//No al max torque
int paso [4][4] =
{
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};
 
void setup()
{
  // Todos los pines se configuran como salida, ya que el motor no enviará señal a Arduino
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  delayTime = 1000 / frequency;
}
 
// Bucle para hacerlo girar
void loop()
{ 
    for (int i = 0; i < 4; i++)
    {
      digitalWrite(IN1, paso[i][0]);
      digitalWrite(IN2, paso[i][1]);
      digitalWrite(IN3, paso[i][2]);
      digitalWrite(IN4, paso[i][3]);
      delay(delayTime);
    }
}