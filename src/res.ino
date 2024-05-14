#include <SPI.h>                                         
#include <nRF24L01.h>                                    
#include <RF24.h>        
//#include <Adafruit_NeoPixel.h>
 

#define SPEED_1      5 
#define DIR_1        4
 
#define SPEED_2      6
#define DIR_2        7

#include <stdio.h> // this is for printf()
#include <stdbool.h> // this is for bool type
#include <math.h> // this is for fabs()

typedef struct 
{
    int left_speed;
    bool left_direction; // 1 - forward, 0 - reverse
    int right_speed;
    bool right_direction; // 1 - forward, 0 - reverse
}
LR_speed;

LR_speed compute_LR_speed (int steer, int speed);

LR_speed compute_LR_speed (int steer, int speed)
{
    const int steer_bias = 488;
    const int speed_bias = 524;
    const int half_max_control_value = 512;
    const int max_speed = 255;
    const float k = 2; // steer susceptibility

    LR_speed speeds;

    float a = (float) (steer - steer_bias) / half_max_control_value;
    if (a > 1)
    {
        a = 1;
    }
    else if (a < -1)
    {
        a = -1;
    }

    float v = (float) (speed - speed_bias) / half_max_control_value;
    if (v > 1)
    {
        v = 1;
    }
    else if (v < -1)
    {
        v = -1;
    }

    float v_L = v * (1 + (k * a));
    if (v_L >= 0 )
    {
        speeds.left_direction = 1;
    }
    else
    {
        speeds.left_direction = 0;
    }

    v_L = fabs (v_L);
    if (v_L > 1)
    {
        v_L = 1;
    }
    v_L = v_L * max_speed;
    speeds.left_speed = (int) v_L;

    float v_R = v * (1 - (k * a));
    if (v_R >= 0 )
    {
        speeds.right_direction = 1;
    }
    else
    {
        speeds.right_direction = 0;
    }

    v_R = fabs (v_R);
    if (v_R > 1)
    {
        v_R = 1;
    }
    v_R = v_R * max_speed;
    speeds.right_speed = (int) v_R;

    return speeds;
}


const uint8_t pinH1   = 7;                                   // Создаём константу указывая номер вывода H1 MotorShield (он управляет направлением 1 мотора)
const uint8_t pinE1   = 6;                                   // Создаём константу указывая номер вывода E1 MotorShield (он управляет скоростью    1 мотора)
const uint8_t pinE2   = 5;                                   // Создаём константу указывая номер вывода E2 MotorShield (он управляет скоростью    2 мотора)
const uint8_t pinH2   = 4;                                   // Создаём константу указывая номер вывода H2 MotorShield (он управляет направлением 2 мотора)
      uint8_t mSpeed  = 0;                                   // Создаём переменную для хранения скорости    моторов
      bool    mDirect = HIGH; 

RF24 radio(8, 9); // nRF24L01+ (CE, CSN)
int data[5]; 

void setup(){
  Serial.begin(9600);
  
  pinMode(pinH1, OUTPUT); digitalWrite(pinH1, LOW);          // Конфигурируем вывод pinH1 как выход и устанавливаем на нём уровень логического «0»
  pinMode(pinE1, OUTPUT); digitalWrite(pinE1, LOW);          // Конфигурируем вывод pinE1 как выход и устанавливаем на нём уровень логического «0»
  pinMode(pinE2, OUTPUT); digitalWrite(pinE2, LOW);          // Конфигурируем вывод pinE2 как выход и устанавливаем на нём уровень логического «0»
  pinMode(pinH2, OUTPUT); digitalWrite(pinH2, LOW); 
  pinMode(3, OUTPUT);
  radio.begin();                                        
  radio.setChannel(5);                                  // Указываем канал приёма данных (от 0 до 127), 5 - значит приём данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
  radio.setDataRate     (RF24_1MBPS);                   // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
  radio.setPALevel      (RF24_PA_HIGH);                 // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
  radio.openReadingPipe (1, 0x1234567890LL);            // Открываем 1 трубу с идентификатором 0x1234567890 для приема данных (на одном канале может быть открыто до 6 разных труб, которые должны отличаться только последним байтом идентификатора)
  radio.startListening  ();                             // Включаем приемник, начинаем прослушивать открытую трубу
  //  radio.stopListening   ();                         // Выключаем приёмник, если потребуется передать данные

  // motor pins
  for (int i = 4; i < 8; i++) {     
    pinMode(i, OUTPUT);
  }
}

void loop(){
    if(radio.available()){                                // Если в буфере имеются принятые данные
        radio.read(&data, sizeof(data));                  // Читаем данные в массив data и указываем сколько байт читать
     
        int xPosition = data[0];
        int yPosition = data[1];
        int btnState = data[2];
        int tmblrState = data[3];
        int ptmrVal = data[4];
        LR_speed speeds = compute_LR_speed (xPosition, yPosition);

        Serial.println("Xj:" + String(xPosition) + "; Yj:" + String(yPosition)); // + "; Btn:" + String(btnState) + "; Tmblr:" + String(tmblrState) + "; Ptr:" + String(ptmrVal));
        Serial.println("left_speed = " + String (speeds.left_speed));
        Serial.println("left_direction = " + String (speeds.left_direction));
        Serial.println("right_speed = " + String (speeds.right_speed));
        Serial.println("right_direction = " + String (speeds.right_direction));
        Serial.println(" ");

        digitalWrite(pinH1, speeds.left_direction);
        analogWrite(pinE1, speeds.left_speed);
        //delay(30);
        digitalWrite(pinH2, speeds.right_direction);
        analogWrite(pinE2, speeds.right_speed);
        delay(30);
        if (tmblrState == 1)
        { 
           digitalWrite(3, HIGH); 
        }
        else                                                                                                  
          digitalWrite(3, LOW);

        
    }
    //colorWipe(matrix.Color(0, 0, 0), 50);
    //Serial.print("1");
}

