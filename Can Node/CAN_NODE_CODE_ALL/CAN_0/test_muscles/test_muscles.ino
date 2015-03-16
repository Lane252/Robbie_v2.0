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

const unsigned int length = 7;
unsigned int message[length]={0,0,0,0,0,0,0};

const unsigned int slength = 7;
unsigned int smessage[slength]={0,0,0,0,0,0,0};


unsigned int freq=50;
unsigned int cyc=0;


int motorR=(2047-((12*2047/15)));
int motorF=(2047+((12*2047/15)));
int stopm=2047;
int armmax=500;

int x= 0;
int y= 2900;
int b=4000;



void setup() {
  Serial.begin(38400);
  Serial.println("16 channel PWM test!");
  mySerial.begin(19200);
  
   

  pwm_motors.begin();
  pwm_motors.setPWMFreq(1000);

  pwm_air.begin();
  pwm_air.setPWMFreq(20);


   

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
  
  
  
  
 if (Serial.available()>=slength)
 {

    
       for(int i=0;i<length;i++)
       {
            smessage[i]=Serial.read();
            smessage[i]=smessage[i]-'0';
            
            
            //Serial.print(smessage[i]);  
            //Serial.print(' ');
           
       }
       Serial.flush();
       freq=smessage[0]*100 + smessage[1]*10 + smessage[2];
       cyc=smessage[4]*100 + smessage[5]*10 + smessage[6];
       
        pwm_air.setPWMFreq(freq);
      
        
       Serial.println(' ');
       Serial.print("freq: ");
       Serial.println(freq);
       Serial.print("cyc: ");
       Serial.println(cyc);


        
 }

         
         

         
 if (mySerial.available()>=length)
 {
     for(int i=0;i<length;i++)
     {
          message[i]=mySerial.read();
         // Serial.print(message[i]);  
         // Serial.print(' ');
     }
  
     if (message[0] != length)
     {
         Serial.println("error, incorrect number of bytes sent or error in timing of reading from serial port");
         for(int i=0;i<length;i++)//assume the timing is wrong and chars are not being read as the correct packet (so length should be one of the other elements) 
         {
               if (message[i]==length)
                   {
                     for(int j=0;j<i;j++)
                         {
                             mySerial.read();
                         }
                         
                     
                   }
             
         }
           
         Serial.println("length not found");
     }      
           
     else
      {mySerial.flush();
  
         pwm_motors.setPWM(12,( map(message[1],0,254,motorR,motorF)), 4095);//compressor
         
         
         int a=((float)cyc/1000)*4095;
         for (int i=1; i<13;i++)
         {
           
           if ( i==message[2])
             {
               pwm_air.setPWM(i,0, a);  
                //Serial.println(a);
               
               /*if(message[2]==3 )//||message[2]==4 || message[2]==7
               
               {
               pwm_air.setPWM(4,0, a);
                pwm_air.setPWM(3,0, a);
                //pwm_air.setPWM(7,0, a);
               
               }
               else if(message[2]==2 )
               {pwm_air.setPWM(2,0, a);
               pwm_air.setPWM(5,0, a);
               //pwm_air.setPWM(6,0, a);
                 
               }
               
               else
               {
                pwm_air.setPWM(i,0, a);  
                //Serial.println(a);
               }
                */
               
             }
             else
             {
           
               pwm_air.setPWM(i, 0, 0);
             }
           
           
         }
         
         
         
         
    
      }
    

  }
  
}
