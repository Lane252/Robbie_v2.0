// control of joints with simple on off control

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

const unsigned int length = 13;
unsigned int message[length]={0,0,0,0,0,0,0,0,0,0,0};
unsigned int actual_length;
//message:
//255
//compressor
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
int gndmax=550;
int uppmax=600;




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
    
  //read CAN
  if(CAN_MSGAVAIL == CAN.checkReceive())            // check if data coming
  { 
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

      tank_pressure=buf[0];
    Serial.println(tank_pressure);

  }

  
  
      
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
       Serial.println ("got valid message");
         if (tank_pressure<110)  pwm_motors.setPWM(12,( map(message[1],0,254,motorR,motorF)), 4095);//compressor
          else pwm_motors.setPWM(12, 0, 4095);
         pwm_motors.setPWM(14,(map(message[2],0,254,stopm-3*armmax,stopm+3*armmax)), 4095);//left arm
         pwm_motors.setPWM(3, (map(message[3],254,0,stopm+2*armmax,stopm-2*armmax)), 4095);  //right arm   
         pwm_motors.setPWM(13,(map(message[4],254,0,stopm-gndmax,stopm+gndmax)), 4095);//left ground wheel  was 2
         pwm_motors.setPWM(2,( map(message[5],254,0,stopm+gndmax,stopm-gndmax)), 4095);//right ground wheel
         
         pwm_motors.setPWM(15,( map(message[6],254,0,stopm-uppmax,stopm+uppmax)), 4095);//left upper wheel  
         pwm_motors.setPWM(1,( map(message[7],0,254,stopm+uppmax,stopm-uppmax)), 4095);//right upper wheel   
         
         //knee
         if(message[8]==254)
         {
          pwm_air.setPWM(4,0,4095);
          pwm_air.setPWM(5,0, 0); 
         }
         
         else if(message[8]==0)
         {
          pwm_air.setPWM(5,0,4095);
          pwm_air.setPWM(4,0, 0); 
         }
         
         else 
         {
           pwm_air.setPWM(4,0, 0);
           pwm_air.setPWM(5,0, 0);    
         }
         
         //hip
         if(message[9]==254)
         {
          pwm_air.setPWM(7,0,4095);
          pwm_air.setPWM(6,0, 0); 
         }
         
         else if(message[9]==0)
         {
          pwm_air.setPWM(6,0,4095);
          pwm_air.setPWM(7,0, 0); 
         }
         
         else 
         {
           pwm_air.setPWM(7,0, 0);
           pwm_air.setPWM(6,0, 0);    
         }
         
         
         
          
           //stabaliser
         if(message[10]==254)
         {
          pwm_air.setPWM(8,0,4095);
          pwm_air.setPWM(10,0, 0); 
         }
         
         else if(message[10]==0)
         {
          pwm_air.setPWM(10,0,4095);
          pwm_air.setPWM(8,0, 0); 
         }
         
         else 
         {
           pwm_air.setPWM(8,0, 0);
           pwm_air.setPWM(10,0, 0);    
         }
         
         
        
      //These sections require clarification
           //ass
           
           if(message[11]==254)
         {
          pwm_air.setPWM(0,0,4095);
          pwm_air.setPWM(1,0, 0); 
         }
         
         else if(message[11]==0)
         {
          pwm_air.setPWM(1,0,4095);
          pwm_air.setPWM(0,0, 0); 
         }
         
         else 
         {
           pwm_air.setPWM(0,0, 0);
           pwm_air.setPWM(1,0, 0);    
         }
           
        
        
          
                 
          //arm extension
          if(message[12]==254)
         {
          pwm_air.setPWM(12,0,4095);
          pwm_air.setPWM(11,0, 0); 
         }
         
         else if(message[12]==0)
         {
          pwm_air.setPWM(11,0,4095);
          pwm_air.setPWM(12,0, 0); 
         }
         
         else 
         {
           pwm_air.setPWM(12,0, 0);
           pwm_air.setPWM(11,0, 0);    
         }
         
         
     
    }

}
