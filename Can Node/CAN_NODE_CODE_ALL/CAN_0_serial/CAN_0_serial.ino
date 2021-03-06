#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <mcp_can.h>
#include <SPI.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(7, 8);



Adafruit_PWMServoDriver pwm_air = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm_motors = Adafruit_PWMServoDriver(0x40);

//const unsigned int length = 25;
//unsigned int message[length]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

const unsigned int length = 11;
unsigned int message[length]={0,0,0,0,0,0,0,0,0,0,0};





int motorR=(2047-((12*2047/15)));
int motorF=(2047+((12*2047/15)));
int stopm=2047;
int armmax=500;


void setup() {
  Serial.begin(115200);
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
{int i;
int j;
  
 while (mySerial.available()<length)
 {
 }//do nothing 
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
           if (message[i]==length)
               {
                 for(j=0;j<i;j++)
                     {
                         mySerial.read();
                     }
                     
                 
               }
         
     }
       
     Serial.println("length not found");
 }
 else
 {// use message9087
  //Serial.println(message[1]);
   //pwm_motors.setPWM(12,motorF, 4095);  //compressor
   // pwm_motors.setPWM(14,( map(message[2],0,254,stopm-armmax,stopm+armmax)), 4095);//left arm
   // pwm_motors.setPWM(3,(map(message[3],0,254,stopm+armmax,stopm-armmax)), 4095);  //right arm 
    pwm_motors.setPWM(12,( map(message[10],0,254,motorR,motorF)), 4095);//compressor
    //pwm_motors.setPWM(13,( map(message[11],0,254,motorR,motorF)), 4095);//right wheel
    //pwm_motors.setPWM(2,( map(message[12],0,254,motorR,motorF)), 4095);//left wheel
    Serial.println(map(message[10],0,254,motorR,motorF));
    Serial.println(motorF);
    //pwm_air.setPWM(12,0, 4095);  //valve 1
   //3 pwm_motors.setPWM(12,motorF, 4095);
    
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
    
  /*case gdWheelsId:
     pwm_motors.setPWM(13,( (int) map(rxBuf[0],0,254,motorR,motorF)), 4095);//left
     pwm_motors.setPWM(2,( (int) map(rxBuf[1],0,254,motorR,motorF)), 4095); //right  
     
     break;
     
   case stairWheelsId:
     pwm_motors.setPWM(15,( (int) map(rxBuf[0],0,254,motorR,motorF)), 4095);//left
     pwm_motors.setPWM(1,( (int) map(rxBuf[1],0,254,motorR,motorF)), 4095);   //right
     
     break;
     
   case kneeHipId:
     
     
     break;
     
    */
       

}

