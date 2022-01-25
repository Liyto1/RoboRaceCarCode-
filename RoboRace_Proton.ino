#include <Servo.h>

Servo servo;

 
/* CONFIG */

#define MaxSpeed 55
#define ServoPort 10
#define SENSOR_FRONT A4
#define SENSOR_RIGHT A3
#define SENSOR_LEFT A5
//#define SENSOR_UP A0
#define BortDist 70
#define BackDist 15 //дистанция при которой маштнапоедит назад
#define SecurityDist 35 // безопасная дистанция боковых датчиков
#define SecurityDistFront 140 //безопасная дистанция среднего датчика
#define StartStopDist 200 // дистанция при которой будет стоять двигатель
#define ForvardSpeed 47
#define BackSpeed 60
#define BRAKEVCC 0 
#define CW  2 //движение  nazad
#define CCW  1 //движение вперёд
#define BRAKEGND 3
#define CS_THRESHOLD 500 // предел показания по току (предел аварийной остановки). (хрен знает какой параметр выставлять) Предполагаю, что 1023 - 30А. В этом скетче
#define K 1// коэф резкого поворота
#define S 0.2 //коэф плавного поворота
#define StateForward 1 // езда вперёд
#define StateObstacles 2 // объезд препядствий
#define StateClash 3 // действия при столкновении
#define StateRidingOnSide 4 // безопасная дистанция до бортов


/* CONFIG */

/* SPECIFICATION */

/*крайнеие физические углы сервы (!!!НЕ ТРОГАТЬ!!!)*/
int frontdata = 0;
int leftdata = 0;
int rightdata = 0;
int updata = 0;
float varVolt = 21.22769;  // среднее отклонение (ищем в excel)
float varProcess = 0.01; // скорость реакции на изменение (подбирается вручную)
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe[3] = { 0.0 };
int DistRight = 0;
int DistLeft = 0;
int DistFront = 0;
int DistUp = 0;
const int IN_A = 4;  // INA: по часовой стрелке. А на деле ток пойдет по проводу А
const int IN_B = 9; // INB: против часовой стрелки. А на деле ток пойдет по проводу В
const int PWM = 6; // PWM: Шим
           //const int cspi = A3; // CS: Показания с датчика тока на моторах    
const int enpin = A1; // EN:что дефис то типа ключа для включения того или иного двигателя2
int ServoFront = 85;
int ServoRight = 60;
int ServoLeft = 110;
int SensorData[3] = { 0 };
int Distans[3] = { 0 };
int oldData = 0;
int state = 1;
/* SPECIFICATION */



//For Analog Sensor

void ReadSensor()
{
  SensorData[2] = analogRead(SENSOR_RIGHT) / 1024.0*5.0 * 1000;
  rightdata = filter(2,SensorData[2]);
  Distans[2] = -2.07456e-14 *rightdata * rightdata * rightdata * rightdata * rightdata  + 1.7974e-10*rightdata * rightdata * rightdata * rightdata - 6.02449e-7 *rightdata * rightdata * rightdata + 0.000986184*rightdata * rightdata - 0.826199 *rightdata + 339.513;
    DistRight = constrain(Distans[2] ,10 , 150);
  
  SensorData[0] = analogRead(SENSOR_LEFT) / 1024.0*5.0 * 1000;
  leftdata = filter(0,SensorData[0]);
  Distans[0] = -1.97695e-14 *leftdata * leftdata * leftdata * leftdata * leftdata + 1.64124e-10*leftdata * leftdata * leftdata * leftdata - 5.31629e-7*leftdata * leftdata * leftdata +0.0008544 *leftdata * leftdata  - 0.725474*leftdata + 317.268;
  DistLeft = constrain(Distans[0] , 10 , 150);
  
  SensorData[1] = analogRead(SENSOR_FRONT) / 1024.0*5.0 * 1000;
  frontdata = filter(1,SensorData[1]);
  Distans[1] = -2.42198e-14*frontdata * frontdata * frontdata * frontdata * frontdata  + 2.40505e-10*frontdata *frontdata * frontdata * frontdata - 9.28044e-7*frontdata *frontdata * frontdata  + 0.00173555 *frontdata * frontdata - 1.59642*frontdata + 638.136;
  DistFront = constrain(Distans[1], 10, 150);
/*SensorData[2] = analogRead(SENSOR_RIGHT) / 1024.0*5.0 * 1000;
  rightdata = filter(2,SensorData[2]);
  Distans[2] = -4.44456e-15 *rightdata * rightdata * rightdata * rightdata * rightdata  + 4.63803e-11*rightdata * rightdata * rightdata * rightdata - 1.90772e-7 *rightdata * rightdata * rightdata + 0.000394846*rightdata * rightdata - 0.436187 *rightdata + 240.764;
    DistRight = constrain(Distans[2] ,10 , 150);
  
  SensorData[0] = analogRead(SENSOR_LEFT) / 1024.0*5.0 * 1000;
  leftdata = filter(0,SensorData[0]);
  Distans[0] = -1.11345e-14 *leftdata * leftdata * leftdata * leftdata * leftdata + 9.9512e-11*leftdata * leftdata * leftdata * leftdata - 3.51464e-7*leftdata * leftdata * leftdata + 0.000621638 *leftdata * leftdata  - 0.585135*leftdata + 280.617;
  DistLeft = constrain(Distans[0] , 10 , 150);
  */
/*SensorData[3] = analogRead(SENSOR_UP) /  1024.0*5.0 * 1000;;
  updata = filter(3, SensorData[3]);
  Distans[3] = 4.09173e-12 * updata * updata * updata * updata * updata - 6.94409e-9 * updata * updata * updata * updata + 3.81761e-6 * updata * updata * updata - 0.000469205 * updata * updata - 0.229402 * updata + 68.9935;
  DistUp = constrain(Distans[3], 5, 30);
  */
  Serial.print(DistLeft);
  Serial.print(" ");
  Serial.print(DistFront);
  Serial.print(" ");
  Serial.print(" ");
  Serial.println(DistRight);

  Serial.println(updata);
}


#pragma region servo
void ServoRotate(int Degree)
{
  if (Degree >= ServoLeft)
  {
    Degree = ServoLeft;
  }
  else if (Degree <= ServoRight)
  {
    Degree = ServoRight;
  }
  servo.write(Degree);
}

int DinamicSpeed()
{
 
  int Speed = 0;
  if (DistFront < SecurityDistFront)
  {
    Speed = (ForvardSpeed / 1.8);
  }
  
  else
  {
    Speed = ForvardSpeed;
  }
  // Serial.println(Speed);
  return Speed;
}
int filter(int i, int val) {  //функция фильтрации
  Pc = P + varProcess;
  G = Pc / (Pc + varVolt);
  P = (1 - G)*Pc;
  Xe[i] = G*(val - Xe[i]) + Xe[i]; // "фильтрованное" значение
  return(Xe[i]);

}
void setup() {
  Serial.begin(115200);
  servo.attach(ServoPort);
  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  //pinMode(SENSOR_UP, INPUT);
  pinMode(IN_A, OUTPUT);//назначение параметров пинов для шилда
  pinMode(IN_B, OUTPUT);
  pinMode(PWM, OUTPUT);


  digitalWrite(IN_A, LOW);
  digitalWrite(IN_B, LOW);


}
void motorGo(int direct, int pwm)//написание какой мотор потом куда двигаться и наконец шим
{
  if (direct <= 3)  // предохранитель, только мотор 0 или 1, а направление 0,1,2,3.
  {
    if (direct <= 1)
      digitalWrite(IN_A, HIGH);
    else
      digitalWrite(IN_A, LOW);


    if ((direct == 0) || (direct == 2))
      digitalWrite(IN_B, HIGH);
    else
      digitalWrite(IN_B, LOW);

    analogWrite(PWM, pwm);
  }
}

void motorBrake()
{
  digitalWrite(IN_A, HIGH);
  digitalWrite(IN_B, HIGH);
  analogWrite(PWM, 1);
}

void motorOff()
{
  digitalWrite(IN_A, LOW); // отключаем питание по ноге А конкретного мотора
  digitalWrite(IN_B, LOW); // отключаем питание по ноге В конкретного мотора
  analogWrite(PWM, 0);    // принудительно подаем 0 на шим конкретного мотора, только зачем? Мотор и так остановится...
}


 void loop() {
  ReadSensor();
  
  //(DistUp > 12)
  //{
    /*if (DistFront > SecurityDistFront)
    {
    if (DistLeft < SecurityDist)
    {
    motorGo(CCW, MaxSpeed);
    ServoRotate(ServoFront - (-0.000142192 *DistLeft * DistLeft * DistLeft + 0.0201082 * DistLeft * DistLeft - 1.01557 * DistLeft + 28.3623));
    }
    else if(DistRight < SecurityDist)
    {
    motorGo(CCW, MaxSpeed);
    ServoRotate(ServoFront + (-0.000142192 *DistRight * DistRight * DistRight + 0.0201082 * DistRight * DistRight - 1.01557 * DistRight + 28.3623));
    }
    }*/
    if (DistFront > SecurityDistFront)
    {
      if (DistLeft - DistRight > 5 || DistRight - DistLeft > 5)
      {
        motorGo(CCW, MaxSpeed);
        ServoRotate(ServoFront + (DistLeft - DistRight)*S);
      }
      /*else
      motorGo(CCW, MaxSpeed);
      ServoRotate(ServoFront);*/
    }

    else if (DistLeft < DistRight && DistFront < 100 && DistRight > SecurityDist)
    {
      motorGo(CCW, DinamicSpeed());
      ServoRotate((ServoFront - (DistFront * (DistRight * 0.7) *0.002 * 50)) - (16.6667 - 0.111 * DistFront));
      Serial.println("Right");
      //Serial.print("ServoRight = ");
      //Serial.println((ServoFront - (DistFront * (DistRight * 0.7) *0.002 * K)) - (16.6667 - 0.111 * DistFront));
    }

    else if (DistLeft > DistRight && DistFront < 100 && DistLeft > SecurityDist)
    {
      motorGo(CCW, DinamicSpeed());
      ServoRotate((16.6667 - 0.111 * DistFront) + (ServoFront + (DistFront * (DistLeft * 0.7) *0.002 * K)));
      Serial.println("Left");
      //  Serial.print("ServoLeft = ");
      //  Serial.println((16.6667 - 0.111 * DistFront) + (ServoFront + (DistFront * (DistLeft * 0.7) *0.002 * K)));
    }


    else if (DistLeft < DistRight && DistFront > SecurityDistFront && (DistLeft - DistRight > 10 || DistRight - DistLeft > 10))
    {
      motorGo(CCW, DinamicSpeed());
      ServoRotate(ServoFront + (DistLeft - DistRight)*S);
      Serial.println("bort");
      }
    else if(DistFront > SecurityDistFront || DistFront < SecurityDistFront && DistRight < SecurityDist && DistLeft < SecurityDist)
    {do{ motorGo(CW, BackSpeed);
    ServoRotate(ServoLeft);
    }
    while ( DistRight > SecurityDist || DistLeft > SecurityDist );
   
    }
  
  //else if (DistUp < 12)
  //{//
    //if (DistUp < 12)
    //{
     // while (DistUp < 20)
      //{
        //ReadSensor();
        //motorGo(CW, BackSpeed);
        //ServoRotate(ServoFront);
        //  Serial.println("prjamo nazad");

 }  
