#include <SPI.h>      //library for SPI protocol                                  
#include <nRF24L01.h>  //libraries for radio connection                                
#include <RF24.h>        
#include <Adafruit_NeoPixel.h>  //library for led-matrix
Adafruit_NeoPixel strip (30, 3, NEO_GRB + NEO_KHZ800);  //object for using led-matrix
 

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

LR_speed compute_LR_speed (int steer, int speed)      //function of computing steer and speed of robot
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


const uint8_t pinH1   = 7;                                   // pins for motor control
const uint8_t pinE1   = 6;                                   
const uint8_t pinE2   = 5;                                   
const uint8_t pinH2   = 4;                                   
      uint8_t mSpeed  = 0;                                   
      bool    mDirect = HIGH; 

RF24 radio(8, 9); // nRF24L01+ (CE, CSN)
int data[5]; 

void setup(){
  Serial.begin(9600);
  
  pinMode(pinH1, OUTPUT); digitalWrite(pinH1, LOW);         
  pinMode(pinE1, OUTPUT); digitalWrite(pinE1, LOW);          
  pinMode(pinE2, OUTPUT); digitalWrite(pinE2, LOW);          
  pinMode(pinH2, OUTPUT); digitalWrite(pinH2, LOW); 
  pinMode(3, OUTPUT);
  radio.begin();                                        
  radio.setChannel(5);                  //set 5 channel   (2.405 Ghz)             
  radio.setDataRate     (RF24_1MBPS);   //set data rate            
  radio.setPALevel      (RF24_PA_HIGH); //set receiver power              
  radio.openReadingPipe (1, 0x1234567890LL);      //openning pipe      
  radio.startListening  ();             //start listening                
  //  radio.stopListening   ();                         

  // motor pins
  for (int i = 4; i < 8; i++) {     
    pinMode(i, OUTPUT);
  }
}

void loop(){
    if(radio.available()){                                // when data are available
        radio.read(&data, sizeof(data));                  
     
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

        digitalWrite(pinH1, speeds.left_direction);        //motor control using received data
        analogWrite(pinE1, speeds.left_speed);
        //delay(30);
        digitalWrite(pinH2, speeds.right_direction);
        analogWrite(pinE2, speeds.right_speed);
        delay(30);
        if (tmblrState == 1)
        { 
           for (int i = 0; i <= 15; i++)
              strip.setPixelColor(i, strip.Color(250, 0, 0));  //turning matrix on with red colour
           strip.show();
        }
        else
        {                                                                                                  
          for (int i = 0; i <= 15; i++)
              strip.setPixelColor(i, strip.Color(0, 0, 0));  //turning matrix off
           strip.show();
        }
        
    }
    //colorWipe(matrix.Color(0, 0, 0), 50);
    //Serial.print("1");
}
