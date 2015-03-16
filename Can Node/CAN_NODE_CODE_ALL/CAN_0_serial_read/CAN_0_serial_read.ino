#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(7, 8);

unsigned char Flag_Recv = 0;
unsigned char len = 0;
unsigned char buf[8];
char str[20];
unsigned char tank_pressureID =0x1D;
int tank_pressure;


Adafruit_PWMServoDriver pwm_air = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm_motors = Adafruit_PWMServoDriver(0x40);

//const unsigned int length = 25;
//unsigned int message[length]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

const unsigned int length = 11;
unsigned int message[length]={
  0,0,0,0,0,0,0,0,0,0,0};





int motorR=(2047-((12*2047/15)));
int motorF=(2047+((12*2047/15)));
int stopm=2047;
int armmax=500;


void setup() {
  Serial.begin(115200);

START_INIT:

  if(CAN_OK == CAN.begin(CAN_500KBPS))                   // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init ok!");
  }
  else
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(100);
    goto START_INIT;
  }

  CAN.init_Mask(0, 0, 0x3ff);                         // there are 2 mask in mcp2515, you need to set both of them
  CAN.init_Mask(1, 0, 0x3ff);

  CAN.init_Filt(0, 0, tank_pressureID);// from CAN2 pressure in tank

  Serial.println("16 channel PWM test!");
  mySerial.begin(19200);

  //CAN.begin(CAN_500KBPS); 

  pwm_motors.begin();
  pwm_motors.setPWMFreq(1000);

  pwm_air.begin();
  pwm_air.setPWMFreq(20);


  uint8_t twbrbackup = TWBR;
  TWBR = 12;   

  pwm_motors.setPWM(1, 2047, 4095);
  pwm_motors.setPWM(2, 2047, 4095);
  pwm_motors.setPWM(3, 2047, 4095);
  pwm_motors.setPWM(12, 2047, 4095);
  pwm_motors.setPWM(13, 2047, 4095);
  pwm_motors.setPWM(14, 2047, 4095);
  pwm_motors.setPWM(15, 2047, 4095);

  //Solenoid Valves
  pwm_air.setPWM(0, 0, 0);
  pwm_air.setPWM(1, 0, 0);
  pwm_air.setPWM(2, 0, 0);
  pwm_air.setPWM(3, 0, 0);
  pwm_air.setPWM(4, 0, 0);
  pwm_air.setPWM(5, 0, 0);
  pwm_air.setPWM(6, 0, 0);
  pwm_air.setPWM(7, 0, 0);

  pwm_air.setPWM(8, 0, 0);
  pwm_air.setPWM(10, 0, 0);
  pwm_air.setPWM(11, 0, 0);
  pwm_air.setPWM(12, 0, 0);
  pwm_air.setPWM(15, 0, 0);


}

void loop() 
{

  //read CAN
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  { 
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

      tank_pressure=buf[0];
    Serial.println(tank_pressure);

  }





  int i;
  int j;

  if (mySerial.available()>=length)
  {

    for(i=0;i<length;i++)
    {
      message[i]=mySerial.read();
      Serial.print(message[i]);  
      Serial.print(' ');
    }
    Serial.println(' ');
    if (message[0] != 255)
    {
      Serial.println("error, incorrect number of bytes sent or error in timing of reading from serial port");
      for(i=0;i<length;i++)//assume the timing is wrong and chars are not being read as the correct packet (so length should be one of the other elements) 
      {
        if (message[i]==255)
        {
          for(j=0;j<i;j++)
          {
            mySerial.read();
          }
          break;   
        }

      }

      Serial.println("length not found");
    }
    else
    {
      //Serial.println(message[1]);
      
      pwm_motors.setPWM(14,( map(message[2],0,254,stopm-armmax,stopm+armmax)), 4095);//left arm
      pwm_motors.setPWM(3,(map(message[3],0,254,stopm+armmax,stopm-armmax)), 4095);  //right arm 
      if (tank_pressure<110) pwm_motors.setPWM(12,( map(message[10],0,254,motorR,motorF)), 4095);//compressor
      else pwm_motors.setPWM(12, 0, 4095);
      pwm_motors.setPWM(13,( map(message[11],0,254,motorR,motorF)), 4095);//right wheel
      pwm_motors.setPWM(2,( map(message[12],0,254,motorR,motorF)), 4095);//left wheel
      Serial.println(map(message[10],0,254,motorR,motorF));
      Serial.println(motorF);

      //pwm_air.setPWM(12,0, 3000*message[10]);  //valve 1
      //pwm_air.setPWM(11,0, 3000*message[1]);  //valve 2
      pwm_air.setPWM(10,0, 3000*message[1]);  //valve 3
      pwm_air.setPWM(8,0, 3000*message[2]);  //valve 4
      pwm_air.setPWM(7,0, 3000*message[3]);  //valve 5
      pwm_air.setPWM(6,0, 3000*message[4]);  //valve 6
      pwm_air.setPWM(5,0, 3000*message[5]);  //valve 7
      pwm_air.setPWM(4,0, 3000*message[6]);  //valve 8
      pwm_air.setPWM(3,0, 3000*message[7]);  //valve 9
      pwm_air.setPWM(2,0, 3000*message[8]);  //valve 10
      pwm_air.setPWM(1,0, 3000*message[9]);  //valve 11
      // pwm_air.setPWM(0,0, 3000*message[10]);  //valve 12


    }
  }

  


}




