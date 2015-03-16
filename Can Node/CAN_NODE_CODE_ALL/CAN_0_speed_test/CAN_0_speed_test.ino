#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <mcp_can.h>
#include <SPI.h>


#define armId 0X11
#define gdWheelsId 0x12
#define stairWheelsId 0x13
#define kneeHipId 0x14

Adafruit_PWMServoDriver pwm_air = Adafruit_PWMServoDriver(0x41);
Adafruit_PWMServoDriver pwm_motors = Adafruit_PWMServoDriver(0x40);

unsigned char len = 0;
unsigned char rxBuf[8];
long unsigned int rxId;


int motorR=2047-(int)((12/15)*2047);
int motorF=2047+(int)((12/15)*2047);

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");
  CAN.begin(CAN_500KBPS); 

  pwm_motors.begin();
  pwm_motors.setPWMFreq(1000);

  pwm_air.begin();
  pwm_air.setPWMFreq(0);


  uint8_t twbrbackup = TWBR;
  TWBR = 12;   

  pwm_motors.setPWM(1, 2047, 4095);
  pwm_motors.setPWM(2, 2047, 4095);
  pwm_motors.setPWM(3, 2047, 4095);
  pwm_motors.setPWM(12, 2047, 4095);
  pwm_motors.setPWM(13, 2047, 4095);
  pwm_motors.setPWM(14, 2047, 4095);
  pwm_motors.setPWM(15, 2047, 4095);
}

void loop() 
{
  CAN.readMsgBuf(&len, rxBuf);              // Read data: len = data length, buf = data byte(s)
  rxId = CAN.getCanId();                    // Get message ID

  switch (rxId){

  case armId :

    // pwm_motors.setPWM(14,( (int) map(rxBuf[0],0,254,motorR,motorF)), 4095);//left
    pwm_motors.setPWM(3,( (int) map(rxBuf[1],0,254,motorR,motorF)), 4095);  //right        
    break;
    
    /*  case gdWheelsId:
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

}

