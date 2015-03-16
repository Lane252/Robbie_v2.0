#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(1000); 
  uint8_t twbrbackup = TWBR;
  TWBR = 12;
  
  pwm.setPWM(1,2048,4095);
  pwm.setPWM(2,2048,4095);
  pwm.setPWM(3,2048,4095);
  pwm.setPWM(4,2048,4095);
  pwm.setPWM(12,0,4095);//compresser
  pwm.setPWM(13,2048,4095);
  pwm.setPWM(14,2048,4095);
  pwm.setPWM(15,2048,4095);
 
 pinMode(A2,INPUT);
 
}
void loop() {
  
 Serial.println(analogRead(A2));
 
}
