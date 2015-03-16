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

const unsigned int length = 13;
unsigned int message[length]={0,0,0,0,0,0,0,0,0,0,0};
unsigned int actual_length;
//message:
//255
//left arm
//right arm
//left wheel ground
//right wheel ground
//left wheel upper
//right wheel upper
//knee
//hip
//stabaliser
//ass
//arm extension







int motorR=(2047-((12*2047/15)));
int motorF=(2047+((12*2047/15)));
int stopm=2047;
int armmax=500;
int gndmax=500;
int uppmax=500;


int knee_ignore=10;//When the input is less than 10 no action is taken (i.e. both valves closed) 
int knee_start_in=((float)405/1000)*4095;//pwm 5
int knee_full_in=((float)415/1000)*4095;
int knee_start_out=((float)416/1000)*4095;//pwm 4
int knee_full_out=((float)425/1000)*4095;

int knee_start_in1=((float)438/1000)*4095;//pwm 3
int knee_full_in1=((float)450/1000)*4095;
int knee_start_out1=((float)410/1000)*4095;//pwm 2
int knee_full_out1=((float)422/1000)*4095;

int hip_ignore=10;//When the input is less than 10 no action is taken (i.e. both valves closed) 
int hip_start_in=((float)454/1000)*4095;
int hip_full_in=((float)470/1000)*4095;
int hip_start_out=((float)466/1000)*4095;
int hip_full_out=((float)485/1000)*4095;

int sta_ignore=10;//When the input is less than 10 no action is taken (i.e. both valves closed) 
int sta_start_in=((float)430/1000)*4095;
int sta_full_in=((float)445/1000)*4095;
int sta_start_out=((float)316/1000)*4095;
int sta_full_out=((float)328/1000)*4095;

int ass_ignore=10;//When the input is less than 10 no action is taken (i.e. both valves closed) 
int ass_start_in=((float)408/1000)*4095;
int ass_full_in=((float)420/1000)*4095;
int ass_start_out=((float)510/1000)*4095;
int ass_full_out=((float)525/1000)*4095;

int arm_ignore=10;//When the input is less than 10 no action is taken (i.e. both valves closed) 
int arm_start_in=((float)379/1000)*4095;
int arm_full_in=((float)390/1000)*4095;
int arm_start_out=((float)494/1000)*4095;
int arm_full_out=((float)508/1000)*4095;


void setup() {
  Serial.begin(115200);
  Serial.println("16 channel PWM test!");
  mySerial.begin(19200);
  
   

  pwm_motors.begin();
  pwm_motors.setPWMFreq(1000);

  pwm_air.begin();
  pwm_air.setPWMFreq(50);


   

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
  
  
  
      
     while (mySerial.available()<length)
     {
     }//do nothing 
     
     
         
      
     for(int i=0;i<length;i++)
     {
          message[i]=mySerial.read();
          Serial.print(message[i]);  
          Serial.print(' ');
     }     
     Serial.println(' ');
     
     
   if (message[0] != 255)
     {
         Serial.println("error, incorrect number of bytes sent or error in timing of reading from serial port");
         for(int i=0;i<length;i++)//assume the timing is wrong and chars are not being read as the correct packet (so length should be one of the other elements) 
         {
               if (message[i]==255)
                   {
                     for(int j=0;j<i;j++)
                         {
                             mySerial.read();
                         }
                         
                     
                   }
             
         }
           
         
     }      
           
     else
     {// use message
     
         pwm_motors.setPWM(12,( map(message[1],0,254,motorR,motorF)), 4095);//compressor
         pwm_motors.setPWM(14,( map(message[2],0,254,stopm-armmax,stopm+armmax)), 4095);//left arm
         pwm_motors.setPWM(3,(map(message[3],0,254,stopm+armmax,stopm-armmax)), 4095);  //right arm   
         pwm_motors.setPWM(2,( map(message[4],0,254,stopm-gndmax,stopm+gndmax)), 4095);//left ground wheel  
         pwm_motors.setPWM(13,( map(message[5],0,254,stopm+gndmax,stopm-gndmax)), 4095);//right ground wheel
         
         pwm_motors.setPWM(2,( map(message[6],0,254,stopm-uppmax,stopm+uppmax)), 4095);//left upper wheel  
         pwm_motors.setPWM(13,( map(message[7],0,254,stopm+uppmax,stopm-uppmax)), 4095);//right upper wheel   
         
         //knee
         
         if(message[8]>(127-knee_ignore) && (message[8]<(127+knee_ignore)))
         {//close both valves
             pwm_air.setPWM(4,0, 0);
             pwm_air.setPWM(5,0, 0);
             pwm_air.setPWM(3,0, 0);
             pwm_air.setPWM(2,0, 0);
             
         }
         else if (message[8]>127)
         {// rotate knee forward
             pwm_air.setPWM(4,0, map(message[8],127+knee_ignore,254,knee_start_in,knee_full_in));  
             pwm_air.setPWM(5,0, 0);
             pwm_air.setPWM(3,0, map(message[8],127+knee_ignore,254,knee_start_in1,knee_full_in1));  
             pwm_air.setPWM(2,0, 0);
         }
          else if (message[8]<127)
         {// rotate knee back (lower hip)
             pwm_air.setPWM(5,0, map(message[8],127-knee_ignore,0,knee_start_out,knee_full_out));  
             pwm_air.setPWM(4,0, 0);
             pwm_air.setPWM(2,0, map(message[8],127-knee_ignore,0,knee_start_out1,knee_full_out1));  
             pwm_air.setPWM(3,0, 0);
         }
        
         //hip
         
         if(message[9]>(127-hip_ignore) && message[9]<(127+hip_ignore))
         {//close both valves
             pwm_air.setPWM(7,0, 0);
             pwm_air.setPWM(6,0, 0);         
         }
         else if (message[9]>127)
         {// rotate hip forward
             pwm_air.setPWM(7,0, map(message[9],127+hip_ignore,254,hip_start_in,hip_full_in));  
             pwm_air.setPWM(6,0, 0);
         }
          else if (message[9]<127)
         {// rotate hip back 
             pwm_air.setPWM(6,0, map(message[9],127-hip_ignore,0,hip_start_out,hip_full_out));  
             pwm_air.setPWM(7,0, 0);
         }
          
           //stabaliser
         
         if(message[10]>(127-sta_ignore) && message[10]<(127+sta_ignore))
         {//close both valves
             pwm_air.setPWM(8,0, 0);
             pwm_air.setPWM(10,0, 0);         
         }
         else if (message[10]>127)
         {// rotate stabilise to raise robot
             pwm_air.setPWM(8,0, map(message[10],127+sta_ignore,254,sta_start_in,sta_full_in));  
             pwm_air.setPWM(10,0, 0);
         }
          else if (message[10]<127)
         {// rotate stabaliser to lower robot
             pwm_air.setPWM(10,0, map(message[10],127-sta_ignore,0,sta_start_out,sta_full_out));  
             pwm_air.setPWM(8,0, 0);
         }   
     /* These sections require clarification
           //ass
           
         if(message[11]>(127-ass_ignore) && message[10]<(127+ass_ignore))
         {//close both valves
             pwm_air.setPWM(0,0, 0);
             pwm_air.setPWM(1,0, 0);         
         }
         else if (message[11]>127)
         {//
             pwm_air.setPWM(0,0, map(message[10],127+ass_ignore,254,ass_start_in,ass_full_in));  
             pwm_air.setPWM(1,0, 0);
         }
          else if (message[11]<127)
         {// 
             pwm_air.setPWM(1,0, map(message[11],127-ass_ignore,0,sta_start_out,ass_full_out));  
             pwm_air.setPWM(0,0, 0);
         }   
                 
        */
          
                 
          //arm extension
         
         if(message[12]>(127-arm_ignore) && message[12]<(127+arm_ignore))
         {//close both valves
             pwm_air.setPWM(12,0, 0);
             pwm_air.setPWM(11,0, 0);         
         }
         else if (message[12]<127)
         {// rotate stabilise to raise robot
             pwm_air.setPWM(12,0, map(message[10],127-arm_ignore,0,arm_start_in,arm_full_in));  
             pwm_air.setPWM(11,0, 0);
         }
          else if (message[12]>127)
         {// rotate stabaliser to lower robot
             pwm_air.setPWM(11,0, map(message[10],127+arm_ignore,254,arm_start_out,arm_full_out));  
             pwm_air.setPWM(12,0, 0);
         }   
         
     
    }

}
