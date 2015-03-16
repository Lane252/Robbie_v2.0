#include <mcp_can.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


//#define OUTPUT_READABLE_YAWPITCHROLL
//#define PRINT_IMU_MESSAGE
//#define PRINT_PRESSURE

MPU6050 mpu;
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
long int timer;
long int imuValues[6]={400,400, 400, 400,400,400};
int valueIndex=0;

long int pressure_timer;
int countp;
float total_pressure0;
float total_pressure1;


Quaternion q;           
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;   
VectorFloat gravity;  
float euler[3];         
float ypr[3];
int yaw=  0; 
int pitch=  0; 
int roll= 0;

int can_offset=45;
int angle_offset=91;
unsigned char IMU1[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char IMUId1 = 0x1F;
boolean sendIMU=true;
boolean IMUsetup=false;

//CAN_OUT
unsigned char Pressure[8];
unsigned char Pressure1Id = 0x1C;
unsigned char Pressure2Id = 0x1D;
int p;

void clearArray(unsigned char (&array)[8])
{
  for(int i=0;i<8;i++)
  {
    array[i]=0;
  }
}

void convertAng(unsigned char& int1, unsigned char& int2, int ang)
{

  if(ang>=255)
  {
    int1=255;
    int2=ang-255;
  }
  else
  {
    int1=ang;
    int2=0;    
  }

}

int IMUAngAdjust(int angle, int zeroVal, boolean reverse)
{  
  int new_ang=angle-zeroVal;
  if (reverse) new_ang=360-new_ang;
  if (new_ang>360) new_ang=new_ang-360;
  if (new_ang<0)   new_ang=360+new_ang;
  return(new_ang);

}



void setup() {
  Serial.begin(9600);
  if(CAN.begin(CAN_500KBPS) ==CAN_OK) { 
    Serial.println("CAN bus initilized sucessfully at 500KBPS"); 
  }
  else  {
    Serial.println("CAN bus initilzation failed");
  }

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
mpu_initalise:
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

 
  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    IMU1[6]=mpuIntStatus;
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    IMU1[7]=3;//IMU is ready to read but has not stabalised yet
    Serial.println("IMU initalised sucessfully");
  } 
  else 
  {

    Serial.print(devStatus);
    Serial.println(F(")"));
    IMU1[7]=(devStatus);
    CAN.sendMsgBuf(IMUId1, 0, 8, IMU1);
    Serial.println("Retry initialise in 5 seconds");
    delay(5000);
    goto mpu_initalise;
  }
  
}




void loop() {

  total_pressure0+=(analogRead(A2)-522.36)/3.1066;
  total_pressure1+=(analogRead(A3)-513.58)/3.0947;
  countp++;
  
   if(millis()-pressure_timer>500)//every 0.5 sec
  {
    p=total_pressure0/countp;
    Pressure[7]=0;
    if(p<-3) Pressure[7]=2;
    if(p<-50) Pressure[7]=1;
    if(p<0) p=0;
    CAN.sendMsgBuf(Pressure1Id, 0, 8, Pressure);
#ifdef PRINT_PRESSURE
    Serial.print(Pressure[0]);
    Serial.print("\t");
#endif

    p=total_pressure1/countp;
    Pressure[7]=0;
    if(p<-3) Pressure[7]=2;
    if(p<-50) Pressure[7]=1;
    if(p<0) p=0;
    CAN.sendMsgBuf(Pressure2Id, 0, 8, Pressure);
#ifdef PRINT_PRESSURE
    Serial.println(Pressure[0]);

#endif



    pressure_timer=millis();
    countp=0;
    total_pressure0=0;
    total_pressure1=0;


  }

  if (dmpReady)
  {
  mpuIntStatus = mpu.getIntStatus();
  IMU1[6]=mpuIntStatus;
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    sendIMU=false;
    Serial.println("reset");


  }
  else if (mpuIntStatus & 0x02) 
  {

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    yaw=    ypr[0]* 180/M_PI + 180;
    pitch=  ypr[1]* 180/M_PI +180; 
    roll=   ypr[2]* 180/M_PI +180;
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    yaw=IMUAngAdjust(yaw, 170, false);
    convertAng(IMU1[0],IMU1[1],(yaw));
    convertAng(IMU1[2],IMU1[3],pitch);
    convertAng(IMU1[4],IMU1[5],roll);

#ifdef OUTPUT_READABLE_YAWPITCHROLL


    Serial.print("ypr\t");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(roll);
    Serial.print("\t");
    Serial.println(IMU1[7]);
#endif





#ifdef PRINT_IMU_MESSAGE
    Serial.print("IMU messages \t");
    Serial.print(IMU1[0]);
    Serial.print("\t");
    Serial.print(IMU1[1]);
    Serial.print("\t");
    Serial.print(IMU1[2]);
    Serial.print("\t");
    Serial.print(IMU1[3]);
    Serial.print("\t");
    Serial.print(IMU1[4]);
    Serial.print("\t");
    Serial.println(IMU1[5]);


#endif 
   if (sendIMU) CAN.sendMsgBuf(IMUId1, 0, 8, IMU1);
      else sendIMU=true ;  

      // Make sure IMU value is stable
      if (!IMUsetup && dmpReady)
      {
        if(millis()-timer>1500)//every second
        {
          if(yaw==imuValues[valueIndex])
          {
            IMUsetup=true;
            IMU1[7]=0; //imuvalues are now okay to use     
          }
          else imuValues[valueIndex]=yaw;
          timer=millis();        
        }      
        valueIndex++;
        if (valueIndex==6)valueIndex=0;
      }
      if (sendIMU) CAN.sendMsgBuf(IMUId1, 0, 8, IMU1);//stop if reset
      else sendIMU=true ;
    }
  }

  delay(10);
}






