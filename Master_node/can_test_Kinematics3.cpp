#include <iostream>
#include <string>
#include "boost/asio.hpp"
#include <boost/thread/thread.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>


#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>


#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstdio>
#include <ctime>
#include <PID_v1.h>
#include <fcntl.h>
#include <fstream>
#include <ctime>


#include "config.hpp"
#include <vector>
using Eigen::MatrixXd;
using Eigen::VectorXd;
// wifi password on robot: C1FBBB2E1

using namespace std;
//using namespace boost;




//CAN socket variables
int s;
int nbytes;
struct sockaddr_can addr;
struct can_frame frame;
struct ifreq ifr;
char *ifname = "can0";
socklen_t len=sizeof(addr);

//UDP socket variables

const int PORT = 9930;
struct sockaddr_in my_addr, cli_addr; //_in
int sockfd, i;
socklen_t slen=sizeof(cli_addr);

const int BUFLEN = 4;
char buf[BUFLEN];


bool constructor = true;

void err(char *str)
{
    perror(str);
    exit(1);
}

//_______________________________________________________________________________________
static struct termios old, newset;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
    tcgetattr(0, &old); /* grab old terminal i/o settings */
    newset = old; /* make new settings same as old settings */
    newset.c_lflag &= ~ICANON; /* disable buffered i/o */
    newset.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
    tcsetattr(0, TCSANOW, &newset); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
    tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
    char ch;
    initTermios(echo);
    ch = getchar();
    resetTermios();
    return ch;
}

/* Read 1 character without echo */
char getch(void)
{
    return getch_(0);
}

/* Read 1 character with echo */
char getche(void)
{
    return getch_(1);
}


class base
{
private:
    boost::mutex m_mutex;
    void setupCanSocket();
    void setupUdpSocket();

    string stripNum(string text)
    {
        char nums[] = "0123456789";

        for (int i = 0; i < strlen(nums); i++)
        {
            text.erase(remove(text.begin(), text.end(), nums[i]), text.end());
        }
        return text;
    }

    bool canInputPer(string className)
    {
        if (className == "canInput")
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool humanInputPer(string className)
    {
        if (className == "humanInput")
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool balanceInputPer(string className)
    {
        if (className == "balance")
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool kinematicsInputPer(string className)
    {
        if (className == "kinematics")
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    string getClass()
    {
        return stripNum(typeid(*this).name());
    }

    bool range(int num, int lower, int upper)
    {
        if (num >= lower && num <= upper)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool range(int num, int lower)
    {
        if (num >= lower)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    //CAN_IN
    static int PRESSURE[13];
    static int PRESSURE_ERROR[13];
    static int ENCODER[7];
    static int IMU[7][3];
    static int IMU_ERROR[7];
    static int ULTRASONIC[4];
    static int STRINGPOT[2];
    static float CURRENT[7];
    static float VOLTAGE[7];
    static int CURRENT_ERROR[7];
    static int VOLTAGE_ERROR[7];


    //CAN_OUT
    static int PWM[30];

    //APP_IN
    static int WHEEL_L;
    static int WHEEL_R;
    static bool CLIMB;
    static int STEPS;

    //SETPOINTS
    static float SETPOINT[11];// LEFT arm, RIGHT arm, left

    //PID
    static double PID_GAINS[11][3];//2 PIDs, 3 values for each

    //PID INPUTS
    static float INPUT[11];

    //PID OUTPUTS
    static float OUTPUT[20];

    //PROCESSED
    static int ANGLE[15];

    //COMPRESSOR
    static int COMPRESSOR;

    //commands from user
    static int COMMAND[11];

    //Kinematics
    static float EE[5][3];//End effector values (x,y,z)


public:

    base()
    {
        if (constructor)
        {
            setupCanSocket();
            setupUdpSocket();
            constructor = false;
        }
    }

    virtual ~base() {}

    void sendCan(unsigned char id, unsigned char (&data)[8]);

    bool valid(int num)
    {
        if (num == -1)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    //CAN_IN
    const int& getP(int elem)
    {
        return PRESSURE[elem];
    }
    void setP(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            PRESSURE[elem] = num;
        }
    }

    const int& getE(int elem)
    {
        return ENCODER[elem];
    }
    void setE(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            ENCODER[elem] = num;
        }
    }

    const int& getIMU(int elem, int pos)
    {
        return IMU[elem][pos];    //const int (&arr)[7][3] = getIMU();
    }
    void setIMU(int num, int elem, int pos)
    {
        if (canInputPer(getClass()))
        {
            IMU[elem][pos] = num;
        }
    }

    const int& getIMUERR(int elem)
    {
        return IMU_ERROR[elem];
    }
    void setIMUERR(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            IMU_ERROR[elem] = num;
        }
    }

    const int& getU(int elem)
    {
        return ULTRASONIC[elem];
    }
    void setU(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            ULTRASONIC[elem] = num;
        }
    }

    const int& getSP(int elem)
    {
        return STRINGPOT[elem];
    }
    void setSP(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            STRINGPOT[elem] = num;
        }
    }

    const float& getV(int elem)
    {
        return VOLTAGE[elem];
    }
    void setV(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            VOLTAGE[elem] = num;
        }
    }

    const float& getI(int elem)
    {
        return CURRENT[elem];
    }
    void setI(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            CURRENT[elem] = num;
        }
    }

    const int& getVERR(int elem)
    {
        return VOLTAGE_ERROR[elem];
    }
    void setVERR(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            VOLTAGE_ERROR[elem] = num;
        }
    }

    const int& getIERR(int elem)
    {
        return CURRENT_ERROR[elem];
    }
    void setIERR(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            CURRENT_ERROR[elem] = num;
        }
    }

    const int& getPERR(int elem)
    {
        return PRESSURE_ERROR[elem];
    }
    void setPERR(int num, int elem)
    {
        if (canInputPer(getClass()))
        {
            PRESSURE_ERROR[elem] = num;
        }
    }

    //CAN_OUT

    //PROCESSED
    const int& getANG(int elem)
    {
        return ANGLE[elem];
    }
    void setANG(int num, int elem)
    {
        ANGLE[elem] = num;
    }


    //APP_IN
    const int& getWL()
    {
        return WHEEL_L;
    }
    void setWL(int num)
    {
        if (humanInputPer(getClass()))
        {
            WHEEL_L = num;
        }
    }

    const int& getWR()
    {
        return WHEEL_R;
    }
    void setWR(int num)
    {
        if (humanInputPer(getClass()))
        {
            WHEEL_R = num;
        }
    }

    const bool& getCLM()
    {
        return CLIMB;
    }
    void setCLM(bool val)
    {
        if (humanInputPer(getClass()))
        {
            CLIMB = val;
        }
    }

    const int& getSTP()
    {
        return STEPS;
    }
    void setSTP(int num)
    {
        if (humanInputPer(getClass()))
        {
            STEPS = num;
        }
    }


    //SETPOINTS
    const float& getSETPOINT(int elem)
    {
        return SETPOINT[elem];
    }
    void setSETPOINT(float num, int elem)
    {
        SETPOINT[elem] = num;
    }

    //INPUT
    const float& getINPUT(int elem)
    {
        return INPUT[elem];
    }
    void setINPUT(float num, int elem)
    {
        INPUT[elem] = num;
    }

    //OUTPUT
    const float& getOUTPUT(int elem)
    {
        return OUTPUT[elem];
    }
    void setOUTPUT(float num, int elem)
    {
        OUTPUT[elem] = num;
    }

    //PID
    const double& getPID(int pid, int gain)
    {
        return PID_GAINS[pid][gain];
    }
    void setPID(int num, int pid, int gain)
    {
        PID_GAINS[pid][gain] = num;
    }

    //COMPRESSOR
    const int& getCOMPRESSOR()
    {
        return COMPRESSOR;
    }
    void setCOMPRESSOR(int num)
    {
        COMPRESSOR = num;
    }

    //OUTPUT
    const int& getCOMMAND(int elem)
    {
        return COMMAND[elem];
    }
    void setCOMMAND(int num, int elem)
    {
        COMMAND[elem] = num;
    }

    //Kinematics
    const float& getEE(int elem, int pos)
    {
        return EE[elem][pos];    //const int (&arr)[5][3] = getIMU();
    }
    void setEE(float num, int elem, int pos)
    {
        if (kinematicsInputPer(getClass()))
        {
            EE[elem][pos] = num;
        }
    }


};


//CAN_IN
int base::PRESSURE[13] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
int base::ENCODER[7] = {-1,-1,-1,-1,-1,-1,-1};
int base::IMU[7][3] = {{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1}};
int base::ULTRASONIC[4] = {-1,-1,-1,-1,};
int base::STRINGPOT[2] = {-1,-1};
int base::IMU_ERROR[7]= {-1,-1,-1,-1,-1,-1,-1};
float base::VOLTAGE[7]= {-1,-1,-1,-1,-1,-1,-1};
float base::CURRENT[7]= {-1,-1,-1,-1,-1,-1,-1};
int base:: VOLTAGE_ERROR[7]= {-1,-1,-1,-1,-1,-1,-1};
int base:: CURRENT_ERROR[7]= {-1,-1,-1,-1,-1,-1,-1};
int base:: PRESSURE_ERROR[13]= {-1,-1,-1,-1,-1,-1,-1};

//CAN_OUT
int base::PWM[30] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,};

//PROCESSED
int base::ANGLE[15] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

//APP_IN
int base::WHEEL_L = -1;
int base::WHEEL_R = -1;
bool base::CLIMB = false;
int base::STEPS = -1;

//SETPOINT
float base::SETPOINT[11] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

//INPUT
float base::INPUT[11] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

//PID
double base::PID_GAINS[11][3]= {{4,0.01,0.0003},{4,0.01,0.0003},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{1,2,0.003},{1,2,0.003},{0,0,0},{0,0,0},{0,0,0}}; //gains for PID

//OUTPUT
float base::OUTPUT[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//COMMAND
int base::COMMAND[11] = {0,0,0,0,0,0,0,0,0,0,0};

//COMPRESSOR
int base::COMPRESSOR = 0;

//Kinematics
float base::EE[5][3]= {{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0}};

void base::setupUdpSocket()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        err("ERROR opening UDP socket");
    }

    /*bzero(&my_addr, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(PORT);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int bindReturn = bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr));
    */

    memset((char *)&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
   my_addr.sin_addr.s_addr = htonl(INADDR_ANY);


    my_addr.sin_port = htons(PORT);
    /*int bindReturn = bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr));

    if (bindReturn < 0)
    {
        err("ERROR binding UDP socket");
    }
*/
    char *ip="192.168.0.5";

    memset((char *)&cli_addr, 0, sizeof(cli_addr));
    cli_addr.sin_family = AF_INET;
    cli_addr.sin_addr.s_addr = inet_addr(ip);
    cli_addr.sin_port = htons(PORT);

        char buff[3]= {'a','d','f'};
       if( sendto(sockfd, buff, 3, 0, (struct sockaddr*) &cli_addr, sizeof(cli_addr))<0)
       {
           err("failed to send");
       }


}


    void base::setupCanSocket()
    {
        if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("Error while opening socket");
        }

        strcpy(ifr.ifr_name, ifname);
        ioctl(s, SIOCGIFINDEX, &ifr);

        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("Error in socket bind");
        }
    }

    void base::sendCan(const unsigned char id, unsigned char (&data)[8])
    {
        frame.can_id = id;
        frame.can_dlc = 8;

        for (int i = 0; i < 8; i++)
        {
            frame.data[i] = data[i];
        }

        nbytes = write(s, &frame, sizeof(struct can_frame));
    }

//_______________________________________________________________________________________

    class canInput : public base
    {
    private:
        int sendPacket( char identifier[],char message[], int lenMessage);

    public:
        void run();
        int IMUAngAdjust(int angle, int zeroVal);


    };
    int YawAdjustment[8]= {0,0,0,0,0,0,0,0};
    bool IMUsetup[8]= {0,0,0,0,0,0,0,0};

    void canInput::run()
    {
        while(1)
        {
            int previousAddr=frame.can_id;
            static char name[3];
            //nbytes = read(s, &frame, sizeof(struct can_frame));
            nbytes=recvfrom(s, &frame, sizeof(struct can_frame), 0, (struct sockaddr*)&addr, &len);


            if (nbytes < 0)
            {
                err("ERROR reading CAN socket");
            }
            if(frame.can_id !=previousAddr)
            {


                switch(frame.can_id)
                {
                    //CAN0
                case 0x0B:
                    setE(frame.data[0], 0);
                    strcpy(name,"R00");
                    break;
                case 0x0C:
                    setIMU(IMUAngAdjust(frame.data[0]+frame.data[1],YawAdjustment[0] ), 0, 0);
                    setIMU(frame.data[2]+frame.data[3], 0, 1);
                    setIMU(frame.data[4]+frame.data[5], 0, 2);
                    setIMUERR(frame.data[7], 0);
                    strcpy(name,"O00");

                    if(!IMUsetup[0] && frame.data[7]==0)
                    {
                        YawAdjustment[0]=200-getIMU(0,0);
                        setIMU(IMUAngAdjust(getIMU(0,0),YawAdjustment[0] ), 0, 0);
                        IMUsetup[0]=true;

                    }
                    break;


                case 0x0D:
                    setU(frame.data[0], 0);
                    strcpy(name,"U00");
                    break;
                case 0x0E:
                    setU(frame.data[0], 1);
                    strcpy(name,"U01");
                    break;
                case 0x0F:
                    setP(frame.data[0], 0);
                    strcpy(name,"P00");
                    break;
                case 0x10:
                    setP(frame.data[0], 1);
                    strcpy(name,"P01");
                    break;

                    //CAN1
                case 0x15:
                    setE(frame.data[0], 1);
                    strcpy(name,"R10") ;
                    break;
                case 0x1E:
                    setI(frame.data[0]/10,0);
                    setIERR(frame.data[7],0);
                    strcpy(name,"C10");
                    break;
                case 0x1B:
                    setIMU(IMUAngAdjust(frame.data[0]+frame.data[1],YawAdjustment[1] ), 1, 0);
                    setIMU(frame.data[2]+frame.data[3], 1, 1);
                    setIMU(frame.data[4]+frame.data[5], 1, 2);
                    setIMUERR(frame.data[7], 1);
                    if(!IMUsetup[1] && frame.data[7]==0)
                    {
                        YawAdjustment[1]=getIMU(1,0)-270;
                        setIMU(IMUAngAdjust(getIMU(1,0),YawAdjustment[1] ), 1, 0);
                        IMUsetup[1]=true;

                    }
                    strcpy(name,"O10");
                    break;

                case 0x17:
                    setU(frame.data[0], 2);
                    strcpy(name,"U10");
                    break;
                case 0x18:
                    setU(frame.data[0], 3);
                    strcpy(name,"U11");
                    break;
                case 0x19:
                    setP(frame.data[0], 2);
                    strcpy(name,"P10");
                    setPERR(frame.data[7], 2) ;
                    break;
                case 0x1A:
                    setP(frame.data[0], 3);
                    strcpy(name,"P11");
                    setPERR(frame.data[7], 3);
                    break;
                case 0x7E:
                    strcpy(name,"T11");
                    break;


                    //CAN2
                case 0x1F:
                    setIMU(IMUAngAdjust(frame.data[0]+frame.data[1],YawAdjustment[2] ), 2, 0);
                    setIMU(frame.data[2]+frame.data[3], 2, 1);
                    setIMU(frame.data[4]+frame.data[5], 2, 2);
                    setIMUERR(frame.data[7], 2);
                    strcpy(name,"O20");
                    if(!IMUsetup[2] && frame.data[7]==0)
                    {
                        YawAdjustment[2]=getIMU(2,0)-66;
                        setIMU(IMUAngAdjust(getIMU(2,0),YawAdjustment[2] ), 2, 0);
                        IMUsetup[2]=true;

                    }
                    break;

                case 0x1C:
                    setP(frame.data[0], 4);
                    strcpy(name,"P20");
                    setPERR(frame.data[7], 3);
                    break;
                case 0x1D:
                    setP(frame.data[0], 5);
                    strcpy(name,"P21");
                    setPERR(frame.data[7], 5);
                    break;

                    //CAN3
                case 0x29:
                    setP(frame.data[0], 6);
                    strcpy(name,"P30");
                    setPERR(frame.data[7], 6);
                    break;
                case 0x2A:
                    setP(frame.data[0], 7);
                    strcpy(name,"P31");
                    setPERR(frame.data[7], 7);
                    break;
                case 0x32:
                    setV(frame.data[0]/10,0);
                    strcpy(name,"V30");
                    setVERR(frame.data[7],0);
                    break;
                case 0x2B:
                    setIMU(IMUAngAdjust(frame.data[0]+frame.data[1],YawAdjustment[3] ), 3, 0);
                    setIMU(frame.data[2]+frame.data[3], 3, 1);
                    setIMU(frame.data[4]+frame.data[5], 3, 2);
                    setIMUERR(frame.data[7], 3);
                    strcpy(name,"O30");
                    if(!IMUsetup[3] && frame.data[7]==0)
                    {
                        YawAdjustment[3]=getIMU(3,0)-270;
                        setIMU(IMUAngAdjust(getIMU(3,0),YawAdjustment[3] ), 3, 0);
                        IMUsetup[3]=true;

                    }
                    break;



                    //CAN4
                case 0x33:
                    setE(frame.data[0], 2);
                    strcpy(name,"R40");
                    break;
                case 0x42:
                    setI(frame.data[0]/10,1);
                    strcpy(name,"I40");
                    setIERR(frame.data[7],1);
                    break;
                case 0x34:
                    setIMU(IMUAngAdjust(frame.data[0]+frame.data[1],YawAdjustment[4] ), 4, 0);
                    setIMU(frame.data[2]+frame.data[3], 4, 1);
                    setIMU(frame.data[4]+frame.data[5], 4, 2);
                    setIMUERR(frame.data[7], 4);
                    strcpy(name,"O40");

                    if(!IMUsetup[4] && frame.data[7]==0)
                    {
                        YawAdjustment[4]=getIMU(4,0)-160;
                        setIMU(IMUAngAdjust(getIMU(4,0),YawAdjustment[4] ), 4, 0);
                        IMUsetup[4]=true;

                    }
                    break;
                case 0x36:
                    setP(frame.data[0], 8);
                    strcpy(name,"P40");
                    setPERR(frame.data[7], 8);
                    break;





                    //CAN5
                case 0x3D:
                    setE(frame.data[0], 3);
                    strcpy(name,"R50");
                    break;
                case 0x3E:
                    setE(frame.data[0], 4);
                    break;
                    strcpy(name,"R51");
                case 0x3F:
                    setIMU(IMUAngAdjust(frame.data[0]+frame.data[1],YawAdjustment[5] ), 5, 0);
                    setIMU(frame.data[2]+frame.data[3], 5, 1);
                    setIMU(frame.data[4]+frame.data[5], 5, 2);
                    setIMUERR(frame.data[7], 5);
                    strcpy(name,"O50");
                    if(!IMUsetup[5] && frame.data[7]==0)
                    {
                        YawAdjustment[5]=getIMU(5,0)-342;
                        setIMU(IMUAngAdjust(getIMU(5,0),YawAdjustment[5] ), 5, 0);
                        IMUsetup[5]=true;

                    }
                    break;

                    //CAN6
                case 0x47:
                    setE(frame.data[0], 5);
                    strcpy(name,"R60");
                    break;
                case 0x48:
                    setSP(frame.data[0], 0);
                    strcpy(name,"S60");
                    break;
                case 0x49:
                    setP(frame.data[0], 9);
                    strcpy(name,"P60");
                    break;
                case 0x4A:
                    setP(frame.data[0], 10);
                    strcpy(name,"P61");
                    break;
                case 0x4B:
                    setIMU(IMUAngAdjust(frame.data[0]+frame.data[1],YawAdjustment[6] ), 6, 0);
                    setIMU(frame.data[2]+frame.data[3], 6, 1);
                    setIMU(frame.data[4]+frame.data[5], 6, 2);
                    setIMUERR(frame.data[7], 6);
                    strcpy(name,"O60");

                    if(!IMUsetup[6] && frame.data[7]==0)
                    {
                        YawAdjustment[6]=getIMU(6,0)-17;
                        setIMU(IMUAngAdjust(getIMU(6,0),YawAdjustment[6] ), 6, 0);
                        IMUsetup[6]=true;

                    }
                    break;

                    //CAN7
                case 0x51:
                    setE(frame.data[0], 6);
                    break;
                case 0x52:
                    setSP(frame.data[0], 1);
                    break;
                case 0x53:
                    setP(frame.data[0], 11);
                    break;
                case 0x54:
                    setP(frame.data[0], 12);
                    break;
                case 0x55:
                    setIMU(frame.data[0]+frame.data[1], 7, 0);
                    setIMU(frame.data[2]+frame.data[3], 7, 1);
                    setIMU(frame.data[4]+frame.data[5], 7, 2);
                    setIMUERR(frame.data[7], 7);
                    break;

                default:
                    cout<<"invalid CAN address \t " <<hex<< frame.can_id<<endl;
                    break;

                }


                sendPacket(name,(char*) frame.data, 8);
            }

            usleep(10);
        }


    }

    int canInput::sendPacket(char identifier[],char message[], int lenMessage)
    {
        if (lenMessage>8) return -100;
        char buf[10]= {-1,-1,-1,-1,-1,-1,-1,-1,};
        buf[0]=identifier[0];
        buf[1]=identifier[1];
        buf[2]=identifier[2];
        for(i=0; i<lenMessage; i++) buf[i+3]=message[i];

        return sendto(sockfd, buf, 11, 0, (struct sockaddr*) &cli_addr, slen);

    }


    int canInput::IMUAngAdjust(int angle, int zeroVal)
    {
        int new_ang=angle-zeroVal;
        if (new_ang>360) new_ang=new_ang-360;
        if (new_ang<0)   new_ang=360+new_ang;
        return(new_ang);

    }

//_______________________________________________________________________________________

    class humanInput : public base
    {
    private:
        int socketCheck;

    public:
        void run();
    };

    void humanInput::run()
    {


        while(1)
        {
            socketCheck = recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr*) &cli_addr, &slen);
            if (socketCheck < 0)
            {
                err("recvfrom");

            }

            /* if (socketCheck>=0) //Command has been received and buf updated???
             {
                updateControl(buf);
             }
            */
            //printf("Received packet from %s:%d\nData: %s\n\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buf);
            cout << buf << endl;
            for (int i = 0; i < BUFLEN; i++)
            {
                buf[i] = 0;
            }
        }
        //close(sockfd);
        usleep(10);
    }
    /*
    void humanInput::updateControl(char buf[])
    {
        int ibuf[]=(int) buf;



    }
    */
//_______________________________________________________________________________________

    class compressor : public base
    {
    private:
        static unsigned char data[8];
        static const unsigned char id;
    public:
        void run();
    };

    void compressor::run()
    {
        while(1)
        {
            if (valid(getP(5)))
            {
                if (getP(5) > 100)
                {
                    setCOMPRESSOR(0);
                    cout<<"here"<<endl;
                }
                else if (getP(5) < 80)
                {
                    setCOMPRESSOR(127);
                }
            }
            sleep(1);
        }
    }

//_______________________________________________________________________________________

    class information : public base
    {
    private:
        void clearTerminal(int num);
        void psiBar(const int& psi, string varName);
        void angDeg(const int& angle, string varName);
        void imuVal(const int &imuY, const int &imuP, const int &imuR, string varName);
        void pidVal(const int &input, const int &setpoint, const int &output, string varName);
        void EEVal(const int &EEx, const int &EEy, const int &EEz, string varName);
        static string scale;
        double val;
        int star;
    public:
        void run();
    };

    string information::scale = "0---10---20---30---40---50---60---70---80---90---100";

    void information::imuVal(const int &imuY, const int &imuP, const int &imuR, string varName)
    {
        //if (valid(imuY) && valid(imuP) && valid(imuR))
        //{
        cout << varName << endl;
        cout << "YAW\tPITCH\tROLL" << endl;
        cout << imuY << "\t" << imuP << "\t" << imuR << endl;
        //}
        //else
        // {
        //   cout << "///" << endl;
        //  cout << "No valid data for " << varName << endl;
        //  cout << "///" << endl;
        // }
    }
    void information::EEVal(const int &EEx, const int &EEy, const int &EEz, string varName)
    {
        //if (valid(imuY) && valid(imuP) && valid(imuR))
        //{
        cout << varName << endl;
        cout << "x\ty\tz" << endl;
        cout << EEx << "\t" << EEy << "\t" << EEz << endl;
        //}
        //else
        // {
        //   cout << "///" << endl;
        //  cout << "No valid data for " << varName << endl;
        //  cout << "///" << endl;
        // }
    }

    void information::pidVal(const int &input, const int &setpoint, const int &output, string varName)
    {

        cout << varName << endl;
        cout << "input\tsetpoint\toutput" << endl;
        cout << input << "\t" << setpoint << "\t" << output << endl;


    }


    void information::psiBar(const int& psi, string varName)
    {
        if (valid(psi))
        {
            val = (scale.length()/100.0) * psi;
            star = floor(val + 0.5);

            cout << varName << endl;
            cout << scale << endl;
            cout << string(star, '#') << string((scale.length()+1)-star,' ') << "PSI: " << psi << string(10, ' ') << endl;
        }
        else
        {
            cout << "///" << endl;
            cout << "No valid data for " << varName << endl;
            cout << "///" << endl;
        }
    }
    void information::angDeg(const int& angle, string varName)
    {
        if (valid(angle))
        {

            cout<<varName<< "Angle: "<< angle<<endl;
        }
        else
        {
            cout << "///" << endl;
            cout << "No valid data for " << varName << endl;
            cout << "///" << endl;
        }
    }
    void information::clearTerminal(int num)
    {
        num *= 3;
        for (int i = 0; i < num; i++)
        {
            cout << "\033[A\033[2K";
        }
        rewind(stdout);
        ftruncate(1,0);
    }

    void information::run()
    {
        while(1)
        {
            clearTerminal(100);
            // psiBar(getP(0), "PRESSURE0");
            // psiBar(getP(1), "PRESSURE1");
            // psiBar(getP(2), "PRESSURE2");
            //psiBar(getP(9), "PRESSURE9");
            /*)
                    imuVal(getIMU(1, 0), getIMU(1, 1), getIMU(1, 2), "Right ARM");
                    imuVal(getIMU(2, 0), getIMU(2, 1), getIMU(2, 2), "UPPER BODY");
                    imuVal(getIMU(3, 0), getIMU(3, 1), getIMU(3, 2), "Left ARM");
                    imuVal(getIMU(4, 0), getIMU(4, 1), getIMU(4, 2), "KNEE");
                    imuVal(getIMU(5, 0), getIMU(5, 1), getIMU(5, 2), "SHANK");
                    imuVal(getIMU(6, 0), getIMU(6, 1), getIMU(6, 2), "STABIL6ISER");

                    angDeg(getANG(0), "STABILISER ");
                    angDeg(getANG(1), "SHANK ");
                    angDeg(getANG(3), "KNEE ");
                    angDeg(getANG(5), "UPPER BODY  ");
                    */
            EEVal(getEE(0, 0), getEE(0, 1), getEE(0, 2), "Left ARM");
            EEVal(getEE(1, 0), getEE(1, 1), getEE(1, 2), "Right ARM");
            cout<<endl;
            angDeg(getANG(1), "Theta1 ");
            angDeg(getANG(5), "Theta2 ");
            angDeg(getANG(7), "Theta3 ");
            angDeg(getANG(9), "Theta4 (L) ");
            angDeg(getANG(11), "Theta5 (R) ");
            angDeg(getANG(6), "Upper ");
            angDeg(getANG(8), "L_Arm ");
            angDeg(getANG(10), "R_Arm ");



            // angDeg(getANG(1), "Theta1 ");
            // angDeg(getANG(5), "Theta2 ");
            //  angDeg(getANG(7), "Theta3 ");
            /*
                    pidVal(getINPUT(1), getSETPOINT(1), getOUTPUT(1), "Right Arm PID");
                    pidVal(getINPUT(0), getSETPOINT(0), getOUTPUT(0), "Left Arm PID");
            */
            sleep(1);
        }
    }
//_______________________________________________________________________________________

//_______________________________________________________________________________________

    class power : public base
    {
    public:
        void run();
    };

    void power::run()
    {
        while(1)
        {
            sleep(1);
        }
    }

//_______________________________________________________________________________________

    class balance : public base
    {
    public:
        void run();
    };

    void balance::run()
    {
        while(1)
        {
            sleep(1);
        }
    }

//_______________________________________________________________________________________

    class PID_CONTROL : public base
    {
    private:
        double Input;
        double Setpoint,Output,P,I,D;
        int elem, pid;
        PID myPID;


    public:
        PID_CONTROL(int);
        void run();
    };

    PID_CONTROL::PID_CONTROL(int pidin) : myPID(&Input, &Output, &Setpoint, getPID(pidin,0), getPID(pidin,1), getPID(pidin,2), 0)
    {
        pid=pidin;
        P=getPID(pid,0);
        I=getPID(pid,1);
        D=getPID(pid,2);

        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(-127,127);
    }

    void PID_CONTROL::run()
    {
        Output=0;
        setOUTPUT(Output, pid);

        while(!valid(getINPUT(pid)))
        {

            usleep(10000);
        }
        sleep(1);
        setSETPOINT(getINPUT(pid),pid);

        while(1)
        {
            double p=getPID(pid,0);
            double i=getPID(pid,1);
            double d=getPID(pid,2);
            if (P != p || I !=i || D !=d)
            {
                P=p;
                I=i;
                D=d;
                myPID.SetTunings(P,I,D);
            }

            Setpoint=getSETPOINT(pid);
            Input=getINPUT(pid);
            myPID.Compute();

            setOUTPUT(Output, pid);
            usleep(10000);
        }
    }

//_______________________________________________________________________________________


class kinematics : public base
{
Eigen::VectorXd Theta_Left;
Eigen::VectorXd Alpha_Left;
Eigen::VectorXd d_Left;
Eigen::VectorXd a_Left;
Eigen::VectorXd Theta_Right;
Eigen::VectorXd Alpha_Right;
Eigen::VectorXd d_Right;
Eigen::VectorXd a_Right;
Eigen::VectorXd ef_g;
Eigen::VectorXd s;
Eigen::MatrixXd p;

const static float RAD= M_PI/180;

private:

//Angle Definitions

//DH Parameters
    //-----------------------------------Alpha-----------d-----------a-----
    /*Joint 1*/const static float       Alp_1=0,       d_1=0,       a_1=350;
    /*Joint 2*/const static float       Alp_2=0,       d_2=0,       a_2=365;
    /*Joint 3*/const static float       Alp_3=0,       d_3=0,       a_3=305;
    /*Joint 4*/const static float       Alp_4=0,       d_4=270,     a_4=0;
    /*Joint 5*/const static float       Alp_5=0,       d_5=0,       a_5=560;
    /*Base  */ const static float       Alp_B=M_PI,    d_B=0,       a_B=0;
    /*Joint 6*/const static float       Alp_6=0,       d_6=-270,     a_6=0;


//Filling Vectors with DH Parameters
void DHMatrix(MatrixXd&T, double alpha, double a, double d, double theta);
void FKine(MatrixXd &p, VectorXd &s,VectorXd theta, VectorXd alpha, VectorXd a, VectorXd d);
void JointClamp(VectorXd H,MatrixXd &H_M, MatrixXd &theta_lim, VectorXd &theta, VectorXd &delta_theta);
void Check_EE(VectorXd &s);
void Check_Goal(VectorXd &ee_g, VectorXd &a);
void JacobianIK(VectorXd ef_g, VectorXd &theta, VectorXd alpha, VectorXd a,VectorXd d,MatrixXd &theta_lim);
void findDeltaTheta(VectorXd &delta_theta, MatrixXd &J, MatrixXd &J_I, VectorXd ef_g, VectorXd ef_c);
void findJ(MatrixXd &J, MatrixXd s, MatrixXd p);
void findTranspose(MatrixXd &J_I, MatrixXd &J);
void findPinv(MatrixXd &J_I, MatrixXd &J);
void DLS_Method(MatrixXd &J_I, MatrixXd &J);


public:

        void run();
};

void kinematics::DHMatrix(MatrixXd&T, double alpha, double a, double d, double theta){
    T.setIdentity();
//DH Matrix Init
T(0,0)=cos(theta);  T(0,1)=-cos(alpha)*sin(theta);  T(0,2)=sin(alpha)*sin(theta);   T(0,3)=a*cos(theta);
T(1,0)=sin(theta);  T(1,1)=cos(theta)*cos(alpha);   T(1,2)=-cos(theta)*sin(alpha);  T(1,3)=a*sin(theta);
T(2,0)=0;           T(2,1)=sin(alpha);              T(2,2)=cos(alpha);              T(2,3)=d;
T(3,0)=0;           T(3,1)=0;                       T(3,2)=0;                       T(3,3)=1;
}
void kinematics::FKine(MatrixXd &p, VectorXd &s,VectorXd theta, VectorXd alpha, VectorXd a, VectorXd d){
    vector<MatrixXd> T;
    MatrixXd TTCurr(4,4); //Current Homogenous Transformation Matrix
    MatrixXd TTPrev(4,4); //Previous Homogenous Transformation Matrix


    TTCurr.setIdentity();

    TTPrev.setIdentity();


    for (int i = 0; i <JOINTS; i++) {
    DHMatrix(TTCurr,  alpha[i], a[i], d[i],theta[i]);

    TTCurr=TTPrev*TTCurr;

    TTPrev=TTCurr;

//Current Joint Angles
    p(0, i) = TTPrev(0,3);

    p(1, i) = TTPrev(1,3);

    p(2, i) = TTPrev(2,3);

    }
//Current End Effector Position
    s(0) = TTPrev(0,3);

    s(1) = TTPrev(1,3);

    s(2) = TTPrev(2,3);
   //***** Needs changing depending on DOF

}
void kinematics::findTranspose(MatrixXd &J_I, MatrixXd &J){
    J_I=J.transpose();
}
void kinematics::findPinv(MatrixXd &J_I, MatrixXd &J){

//cout<<"Pinv"<<endl;
    Eigen::JacobiSVD<MatrixXd> svd(J, Eigen::ComputeFullU|Eigen::ComputeFullV);
    VectorXd s = svd.singularValues();
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();

    MatrixXd S(s.rows(), V.rows());
    for (int i = 0; i < s.rows(); i++) {
        S(i,i) = s(i);
    }
    J_I = V * (S.transpose() * U.transpose());
}
void kinematics::findJ(MatrixXd &J, MatrixXd s, MatrixXd p){
//cout<<"FindJ"<<endl;
    for (int j = 0; j < J.cols(); j++) {
        Eigen::Vector3d d = s.col(0) - p.col(j);
        J(0,j) = d(2) - d(1) ;
        J(1,j) = d(0) - d(2) ;
        J(2,j) = d(1) - d(0) ;

    }
}
void kinematics::DLS_Method(MatrixXd &J_I, MatrixXd &J){
    MatrixXd I(3,3);
    I << 100,100,100,
         100,100,100,
         100,100,100;

    MatrixXd inver = J*J.transpose() + I;
    J_I = J.transpose() * inver.inverse();
}
void kinematics::JointClamp(VectorXd H,MatrixXd &H_M, MatrixXd &theta_lim, VectorXd &theta, VectorXd &delta_theta){
//cout<<"Joint Clamp"<<endl;

 for(int i=0;i<JOINTS;i++)
        {

            if((theta(i)>theta_lim(i,0)))
            {
                H(i)=0;
            }
            else if (theta(i)<theta_lim(i,1))
            {
                H(i)=0;
            }
            else
            {
                H(i)=1;
            }
        }

        H_M<<H(0),0,0,
             0,H(1),0,
             0,0,H(2);
             //0,0,0,H(3);

        //cout<<H_M<<endl;
        theta = theta + H_M*delta_theta;
        //Make Sure Joints 0>theta>360
        for(int i=0;i<JOINTS;i++)
        {
        if(theta(i)<0)
        {
            theta(i)=(2*M_PI)+theta(i);
        }
        else if (theta(i)>(2*M_PI))
        {
            theta(i)=theta(i)-(2*M_PI);
        }
        else
        { theta(i)=theta(i);
        }
        }
}
void kinematics::Check_EE(VectorXd &s){
 const static float EE_Tol=0.001;
for(int i=0;i<3;i++)
{

    if (abs(s(i)) <(EE_Tol ))

    {
        s(i)=0;
        //cout<<"ABS: "<<abs(s(i))<<endl;
        //s(i)=0;
    }
    else{
    s(i)=s(i);
    }

    //cout<<"S: "<<endl<<s<<endl;
}

}
void kinematics::Check_Goal(VectorXd &ee_g, VectorXd &a){
float Goal_Dist=0;
float Max_Dist=0;

Goal_Dist=sqrt((pow(ee_g(0),2))+((pow(ee_g(1),2))));

for(int i=0;i<JOINTS;i++){
Max_Dist=Max_Dist+a(i);
}
cout<<"Max_Dist"<<Max_Dist<<endl;

if(Goal_Dist>Max_Dist)
{
    cout<<"Target out of Reach"<<endl;

}
cout<<"Target in reach"<<endl;

}
void kinematics::findDeltaTheta(VectorXd &delta_theta, MatrixXd &J, MatrixXd &J_I, VectorXd ef_g, VectorXd ef_c){
//cout<<"Delta Theta"<<endl;
    //findTranspose(J_I, J);
    // findPinv(J_I, J);
    DLS_Method(J_I,J);
    VectorXd delta_e = ef_g - ef_c;
    double error = 999999;
    MatrixXd eye(J.rows(), J_I.cols());
    eye.setIdentity();
    while (error > TOLERANCE) {
        VectorXd err = (eye - (J*J_I)) * delta_e;
        error = err.squaredNorm();
        delta_e /= 2;
    }
    delta_theta = J_I * delta_e;
}
void kinematics::JacobianIK(VectorXd ef_g, VectorXd &theta, VectorXd alpha, VectorXd a,VectorXd d,MatrixXd &theta_lim){
    ofstream Theta;

    Theta.open ("_Theta.txt");
    //Theta2.open ("_Theta2.txt");
    //Theta3.open ("_Theta3.txt");
    //Iteration.open("_Iteration.txt");
    //cout<<"JacobianIK"<<endl;

    int iter = 0;  //

    MatrixXd J(3,JOINTS); // Jacobian matrix

    MatrixXd J_I(JOINTS,3); // Jacobian pseudoinverse matrix

    VectorXd delta_theta(JOINTS); // the change in theta which is to be calculated at each iteration

    VectorXd s(3); // current end effector position

    MatrixXd p(3,JOINTS); // current Joint positions

    VectorXd H(JOINTS);
    MatrixXd H_M(JOINTS,JOINTS);


    //NEED MANUALLY CHANGING DEPENDING ON NUMBER OF JOINTS
    p << 0,0,0,
        0,0,0,
        0,0,1;
        //0,0,0; //Init of joint variables, requires 3 numbers for each joint.
    s << 0,0,0;             //Init of end effector position, requires one number for each joint.


    FKine(p,s, theta, alpha,a,d);
   // Check_EE(s);
    //cout<<"Here4"<<endl;
    VectorXd diff_ee(3);
    diff_ee << 99999.0,99999.0,99999.0;
    diff_ee = s - ef_g;


   cout << "Starting End Effector position: " << endl << s << endl;
    //Newtons method
    while(diff_ee.squaredNorm() > EF_ERROR_TOLERANCE && iter <  MAX_ITER) {
        //cout<<"Here5"<<endl;
        findJ(J,s, p); // this populates J.
        //Check_EE(s);
  //      cout<<"Here6"<<endl;
        findDeltaTheta(delta_theta, J, J_I, ef_g, s);
        //Check_EE(s); // finds the delta thetas for each joint angles needed to get to the goal

       // theta = theta + delta_theta; // update value of theta
       JointClamp(H,H_M,theta_lim,theta,delta_theta);

       //Grad_Proj(H,H_M,theta_lim,theta,delta_theta);
//        cout<<"Here7"<<endl;
        Theta<<iter<<"\t"<< theta(0)*DEG<<"\t "<<theta(1)*DEG<<"\t"<<theta(2)*DEG<<endl;

        FKine(p, s, theta, alpha, a,d); // update v

        diff_ee = s - ef_g;

        iter++;

        //cout<<"iteration: "<<iter<<endl;
        //cout << "End Effector Position: " << endl << s << endl;

        if(iter>=MAX_ITER){

            cout<<"Iteration Limit Reached: "<<iter<<endl;
            cout<<"Could not find Valid Solution"<<endl;
            cout << "Final End Effector Position: " << endl << s << endl;
            std::cout << "Joint Angles:" <<endl<< theta << endl;
            return;
        }
    }
    Theta.close();

   // Check_EE(s);
    cout << "Final End Effector Position: " << endl << s << endl;
    std::cout << "Joint Angles:" <<endl<< theta << endl;
}

void kinematics::run(){
    //cout<<"Run"<<endl;

    Theta=Eigen::VectorXd(JOINTS);
    Alpha=Eigen::VectorXd(JOINTS);
    d=Eigen::VectorXd(JOINTS);
    a=Eigen::VectorXd(JOINTS);
    Theta_Lim=Eigen::MatrixXd(JOINTS,2);
    ef_g=Eigen::VectorXd(3);
    s=Eigen::VectorXd(3); // current end effector position
    p=Eigen::MatrixXd(3,JOINTS);

    ef_g << EE_x,EE_y,EE_z;

    p << 0,0,0,
         0,0,1,
         0,0,0;
         //0,0,1;  // Need 3 numbers for each joint, can be random values

    s << 0,0,0;

    Theta << Th_1,Th_2,Th_3;//Th_3,Th_4;//***************/
    Theta=Theta*RAD;
    Alpha << Alp_1,Alp_2,Alp_3;//**************
    Alpha=Alpha*RAD;
    d     << d_1,d_2,d_3;//,d_3,d_4;//*************
    a     << a_1,a_2,a_3;//*************
    Theta_Lim<<Th_1_MAX,Th_1_MIN,Th_2_MAX,Th_2_MIN,Th_3_MAX,Th_3_MIN;
    Theta_Lim=Theta_Lim*RAD;

    //Function Declaration

    Check_Goal(ef_g,a);
    FKine( p,s,Theta,Alpha,a, d);
    //Check_EE(s);
    cout<<"Here231"<<endl;

    cout<<"EE Position: "<<endl<<s<<endl;
    //INVERSE KINEMATICS
    cout<<"Starting Joint Values: "<<endl;
    cout << "Angle 1" <<endl<< Theta(1)*DEG << endl;
    cout << "Angle 2" <<endl<< Theta(2)*DEG << endl;
   // cout << "Angle 3" <<endl<< Theta(3)*DEG << endl;
    cout << "Angle Base" <<endl<< Theta(0)*DEG << endl;

    JacobianIK(ef_g, Theta, Alpha,a,d,Theta_Lim);

    cout<<"Final Joint Values: "<<endl;
    cout << "Angle 1" <<endl<< Theta(1)*DEG << endl;
    cout << "Angle 2" <<endl<< Theta(2)*DEG << endl;
    //cout << "Angle 3" <<endl<< Theta(3)*DEG << endl;
    cout << "Angle Base" <<endl<< Theta(0)*DEG << endl;
}


/*
void kinematics::run(){
    unsigned long start=(std::clock()/((double) CLOCKS_PER_SEC));



    while(1)
    {
        Theta_Left=Eigen::VectorXd(JOINTS);
        Alpha_Left=Eigen::VectorXd(JOINTS);
        d_Left=Eigen::VectorXd(JOINTS);
        a_Left=Eigen::VectorXd(JOINTS);
        Theta_Right=Eigen::VectorXd(JOINTS);
        Alpha_Right=Eigen::VectorXd(JOINTS);
        d_Right=Eigen::VectorXd(JOINTS);
        a_Right=Eigen::VectorXd(JOINTS);



        //Shank Angle
        setANG(getIMU(6,0),0);

         //SHANK//->Theta1
        setANG(360-getIMU(6,0),1);
        //Stabiliser-Shank angle
        setANG(180-(180-getANG(0))-getANG(1),3);
        //Init Thigh Angle ->
        setANG(getIMU(4,0),4);
        //SHANK-THIGH ANGLE->Theta2
        setANG(360 +getANG(0)-getANG(4),5);
        //UpperBody Angle
        setANG(getIMU(2,0),6);
        //KNEE-UPPERBODY ANGLE->Theta3
        setANG(180-(getANG(6)+180-getANG(4)),7);
        //cout<<getANG(7)<<" Theta3"<<endl;

        //Theta4-Upper-Left Arm
        setANG(getIMU(3,0),8);//imu ang reading
         int larmAng=getANG(6)-getANG(8);
        if (larmAng<0){larmAng=360+larmAng;}
        setANG(larmAng,9);

        //Theta5-Upper-Right Arm
        setANG(getIMU(1,0),10);
        int rarmAng=getANG(6)-getANG(10);
        if (rarmAng<0){rarmAng=360+rarmAng;}
        setANG(rarmAng,11);


        Theta_Left << 0,getANG(1)*RAD,getANG(5)*RAD,getANG(7)*RAD,0,getANG(9)*RAD;
        Alpha_Left << Alp_B,Alp_1,Alp_2,Alp_3,Alp_4,Alp_5;
        d_Left     << d_B,d_1,d_2,d_3,d_4,d_5;
        a_Left     << a_B,a_1,a_2,a_3,a_4,a_5;

        Theta_Right<< 0,getANG(1)*RAD,getANG(5)*RAD,getANG(7)*RAD,0,getANG(11)*RAD;
        Alpha_Right << Alp_B,Alp_1,Alp_2,Alp_3,Alp_6,Alp_5;
        d_Right     << d_B,d_1,d_2,d_3,d_6,d_5;
        a_Right     << a_B,a_1,a_2,a_3,a_6,a_5;

        s=Eigen::VectorXd(JOINTS); // current end effector position
        p=Eigen::MatrixXd (3,JOINTS); // current Joint positionsEigen::VectorXd ef_g;
        p << 0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0; // Need 3 numbers for each joint, can be random values
        s << 0,0,1,0,0,0;
        FKine( p,s,Theta_Left,Alpha_Left,a_Left, d_Left);
        //cout<<"EE Position Left Arm: "<<endl<<s<<endl;
		setEE(s(0),0,0);
		setEE(s(1),0,1);
		setEE(s(2),0,2);
        FKine( p,s,Theta_Right,Alpha_Right,a_Right, d_Right);
        //cout<<"EE Position Right Arm: "<<endl<<s<<endl;
		setEE(s(0),1,0);
		setEE(s(1),1,1);
		setEE(s(2),1,2);





        usleep(10000);
    }
}
*/
//_______________________________________________________________________________________

//_______________________________________________________________________________________



//,
    class serialOutput : public base
    {
    private:
        //static const unsigned int length =13;
        static unsigned char messageone[5];
        static unsigned char messagetwo[4];
        static unsigned char messagethree[4];

        boost::asio::io_service io;
        boost::asio::serial_port serial;

    public:

        void run();
        serialOutput(int a): io(), serial(io,"/dev/ttyO4")//tx is on pin 13
        {


            serial.set_option(boost::asio::serial_port_base::baud_rate(19200));
        }

    };
    unsigned char serialOutput::messageone[5]= {255, 127,127,127,127};
    unsigned char serialOutput::messagetwo[4]= {127,127, 127,127};
    unsigned char serialOutput::messagethree[4]= {127,127,127,127};



    void serialOutput::run()
    {
        int i=0;
        int j=0;

        while(1)
        {
            //MOTORS
            messageone[1]=getCOMPRESSOR()+127;//compressor
            messageone[2]=(int)getOUTPUT(0)+127;//left arm
            messageone[3]=(int)getOUTPUT(1)+127;//right arm
            messageone[4]=(int)getOUTPUT(2)+127;//ground wheels left
            messagetwo[0]=(int)getOUTPUT(3)+127;//ground wheel right
            messagetwo[1]=(int)getOUTPUT(4)+127;//stair wheel left
            messagetwo[2]=(int)getOUTPUT(5)+127;//stair wheel right
            messagetwo[3]=(int)getOUTPUT(6)+127;//knee
            messagethree[0]=(int)getOUTPUT(7)+127;//hip
            messagethree[1]=(int)getOUTPUT(8)+127;//stabaliser
            messagethree[2]=(int)getOUTPUT(9)+127;//ass
            messagethree[3]=(int)getOUTPUT(10)+127;//arm extension



            //cout<<(int)messageone[0] <<"\t"<<(int)messageone[1] <<"\t"<<(int)messageone[2] <<"\t"<<(int)messageone[3] <<"\t"<<(int)messageone[4] <<"\t"<<(int)messagetwo[0] <<"\t"<<(int)messagetwo[1] <<"\t"<<(int)messagetwo[2] <<"\t"<<(int)messagetwo[3] <<"\t"<<(int)messagetwo[0] <<"\t"<<(int)messagetwo[1] <<"\t"<<(int)messagetwo[2] <<"\t"<<(int)messagetwo[3] << endl;
            // sleep(0.01);


            try
            {
                //boost::asio::write(serial,boost::asio::buffer(&message,length));
                boost::asio::write(serial,boost::asio::buffer(&messageone,5));
                boost::asio::write(serial,boost::asio::buffer(&messagetwo,4));
                boost::asio::write(serial,boost::asio::buffer(&messagethree,4));
                i=i+1;
                //cout <<i<<"\t"<< j<<endl;

            }
            catch(boost::system::system_error& e)
            {
                j =j+1;
                cout<<"serial send did not work "<<j<<endl;
                //return ;
            }



            usleep(10000);
        }

        //close(tty_fd);
        //tcsetattr(STDOUT_FILENO,TCSANOW,&old_stdio);
    }
//___________________________________________________________________________________

//,

//
    class remoteControl : public base
    {
    private:
        int wheel_drive;// 1 means drive ground wheels, 2 upper wheels, 3 both sets

        char c;
    public:
        void run();
    };

    void remoteControl::run()
    {
        wheel_drive=1;
        int adj=50;
        int max=100;
        while(1)
        {



            c=getche();


            switch(c)
            {
            case ' ':
                cout<<"\nkill\n";
                setCOMPRESSOR(0);
                setCOMMAND(0,2);
                setCOMMAND(0,3);
                setCOMMAND(0,4);
                setCOMMAND(0,5);
                setCOMMAND(0,1);
                setCOMMAND(0,6);
                setCOMMAND(0,7);
                setCOMMAND(0,8);
                setCOMMAND(0,0);
                setCOMMAND(0,9);
                setCOMMAND(0,10);

                sleep(2);
                return;
                break;

            case 'v':
                cout<<"ground wheels"<<endl;
                wheel_drive=1;
                setCOMMAND(0,2);
                setCOMMAND(0,3);
                setCOMMAND(0,4);
                setCOMMAND(0,5);
                break;

            case 'r':
                cout<<"upper wheels"<<endl;
                wheel_drive=2;
                setCOMMAND(0,2);
                setCOMMAND(0,3);
                setCOMMAND(0,4);
                setCOMMAND(0,5);
                break;

            case 'f':
                cout<<"upper and ground wheels"<<endl;
                wheel_drive=3;
                setCOMMAND(0,2);
                setCOMMAND(0,3);
                setCOMMAND(0,4);
                setCOMMAND(0,5);
                break;


            case 'w':
                cout<<"forward"<<endl;
                if(wheel_drive==1||wheel_drive==3)
                {
                    setCOMMAND(max,2);
                    setCOMMAND(max,3);

                }

                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(max,4);
                    setCOMMAND(max,5);

                }

                break;

            case 'a':
                cout<<"left turn"<<endl;
                if(wheel_drive==1||wheel_drive==3)
                {
                    cout <<"ground wheels"<< endl;
                    setCOMMAND(-adj,2);
                    setCOMMAND(adj,3);
                }
                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(-adj,4);
                    setCOMMAND(adj,5);
                }
                break;

            case 'x':
                cout<<"reverse"<<endl;
                if(wheel_drive==1||wheel_drive==3)
                {
                    setCOMMAND(-max,2);
                    setCOMMAND(-max,3);
                }
                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(-max,4);
                    setCOMMAND(-max,5);
                }
                break;

            case 'd':
                cout<<"turn right"<<endl;
                if(wheel_drive==1||wheel_drive==3)
                {
                    setCOMMAND(adj,2);
                    setCOMMAND(-adj,3);
                }
                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(adj,4);
                    setCOMMAND(-adj,5);
                }
                break;

            case 'q':
                cout<<"drift left"<<endl;
                if(wheel_drive==1||wheel_drive==3)
                {
                    setCOMMAND(max-adj,2);
                    setCOMMAND(max,3);
                }
                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(max-adj,4);
                    setCOMMAND(max,5);
                }
                break;

            case 'e':
                cout<<"drift right"<<endl;
                if(wheel_drive==1||wheel_drive==3)
                {
                    setCOMMAND(max,2);
                    setCOMMAND(max-adj,3);
                }
                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(max,4);
                    setCOMMAND(max-adj,5);
                }
                break;

            case 's':
                cout<<"stop"<<endl;

                if(wheel_drive==1||wheel_drive==3)
                {
                    setCOMMAND(0,2);
                    setCOMMAND(0,3);
                }
                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(0,4);
                    setCOMMAND(0,5);
                }
                break;


            case 'c':
                cout<<"drift right backwards"<<endl;
                if(wheel_drive==1||wheel_drive==3)
                {
                    setCOMMAND(-max,2);
                    setCOMMAND(-max+adj,3);
                }
                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(-max,4);
                    setCOMMAND(-max+adj,5);
                }
                break;

            case 'z':
                cout<<"drift left backwards"<<endl;
                if(wheel_drive==1||wheel_drive==3)
                {
                    setCOMMAND(-max+adj,2);
                    setCOMMAND(-max,3);
                }
                if(wheel_drive==2||wheel_drive==3)
                {
                    setCOMMAND(-max+adj,4);
                    setCOMMAND(-max,5);
                }
                break;

                //Muscle Control
                //hip
            case 'y':
                cout<<"hip forward"<<endl;
                setOUTPUT(127,7);
                break;

            case 'n':
                cout<<"hip back"<<endl;
                setOUTPUT(-127,7);
                break;

            case 'h':
                cout<<"hip stop"<<endl;
                setOUTPUT(0,7);
                break;

                //knee
            case 'i':
                cout<<"knee forward"<<endl;
                setOUTPUT(127,6);
                break;

            case ',':
                cout<<"knee back"<<endl;
                setOUTPUT(-127,6);
                break;

            case 'k':
                cout<<"knee back"<<endl;
                setOUTPUT(0,6);
                break;

                //stabaliser
            case 'u':
                cout<<"stabaliser down"<<endl;
                setOUTPUT(127,8);
                break;
            case 'j':
                cout<<"stabaliser stop"<<endl;
                setOUTPUT(0,8);
                break;
            case 'm':
                cout<<"stabaliser up"<<endl;
                setOUTPUT(-127,8);
                break;

                //left arm
            case 'o':
                cout<<"left arm up"<<endl;
                setOUTPUT(127,0);
                break;

            case 'l':
                cout<<"left arm down"<<endl;
                setOUTPUT(-127,0);
                break;

                //right arm
            case 'p':
                cout<<"right arm up"<<endl;
                setOUTPUT(127,1);
                break;

            case ';':
                cout<<"right arm down"<<endl;
                setOUTPUT(-127,1);
                break;

            case '.':
                cout<<"stop arms"<<endl;
                setOUTPUT(0,1);
                setOUTPUT(0,0);
                break;
                //ass
            case 't':
                cout<<"ass up"<<endl;
                setOUTPUT(127,9);
                break;

            case 'b':
                cout<<"ass down"<<endl;
                setOUTPUT(-127,9);
                break;

            case 'g':
                cout<<"ass stop"<<endl;
                setOUTPUT(0,9);
                break;

                //arm extension
            case ']':
                cout<<"extend arms"<<endl;
                setOUTPUT(127,10);
                break;

            case '[':
                cout<<"retract arms"<<endl;
                setOUTPUT(-127,10);
                break;

            case '#':
                cout<<"stop arms"<<endl;
                setOUTPUT(0,10);
                break;






            }



            usleep(10000);
        }
    }

//_______________________________________________________________________________________

    class main_controller : public base
    {

    private:
        float previous[11];

        int now;
    public:
        void run();
        float adjust(float current, int max,int min, int adj, double time, int com);// returns the new setpoint
        float adjustAng(float current, int max,int min, int adj, double time, int com);


    };
//float main_controller::previous[11]={0,0,0,0,0,0,0,0,0,0,0};

    void main_controller::run()
    {
        for(int i=0; i<11; i++)
        {
            previous[i]=0;
        }
        // last_time = (std::clock()/(((double) CLOCKS_PER_SEC)));//ms

        setINPUT(getANG(10),1);//right arm
        setINPUT(getANG(11),0);//left arm
        setINPUT(getANG(5),7);//hip


        while(1)
        {



            setOUTPUT(adjust(getOUTPUT(2), 127, -127, 300,0.05,getCOMMAND(2)),2);//left wheel
            setOUTPUT(adjust(getOUTPUT(3), 127, -127, 300,0.05,getCOMMAND(3)),3);//right wheel
            setOUTPUT(adjust(getOUTPUT(4), 127, -127, 300,0.05,getCOMMAND(4)),4);//left wheel
            setOUTPUT(adjust(getOUTPUT(5), 127, -127, 300,0.05,getCOMMAND(5)),5);//right wheel

            usleep(50000);


        }
    }


    /*setSETPOINT(adjustAng(getSETPOINT(0), 60, 20, 7,(int)(time-last_time),getCOMMAND(0)),0);//left arm
    setSETPOINT(adjustAng(getSETPOINT(1), 60, 20, 7,time-last_time,getCOMMAND(1)),1);//right arm

    setSETPOINT(adjustAng(getSETPOINT(6), 60, 20, 7,time-last_time,getCOMMAND(6)),6);//knee
    setSETPOINT(adjustAng(getSETPOINT(7), 60, 20, 7,time-last_time,getCOMMAND(7)),7);//hip
    setSETPOINT(adjustAng(getSETPOINT(8), 60, 20, 7,time-last_time,getCOMMAND(8)),8);//stabaliser

    setOUTPUT(adjustAng(getSETPOINT(9), -127, 127, 200,time-last_time,getCOMMAND(9)),9);//ass
    setOUTPUT(adjustAng(getSETPOINT(10), -127, 127, 200,time-last_time,getCOMMAND(10)),10);//arm extension

    setINPUT(getANG(10),1);//right arm
    setINPUT(getANG(11),0);//left arm
    setINPUT(getANG(4),6);//knee
    setINPUT(getANG(5),7);//hip
    setINPUT(getANG(2),8);//stabaliser


    //if a joint has just been told to stop, make the setpoint equal the current position
    for(int i=0; i<11;i++)
    {   now=getCOMMAND(i);
        if (now==0 && previous[i] !=0)setSETPOINT(getINPUT(i),i);

        previous[i]=now;
    }

    last_time=time;
    */



    float main_controller::adjust(float current, int max, int min, int adj, double time, int com)
    {
        //com has range -100 to 100. -100 means move to minimum at fastest rate, 100 means move to max.
        // max output is symeterical about 0 point
        int aim=0;
        if (com>0) aim=((double)com/100)*(max);
        else aim=((double)com/100)*-1*(min);
        double adjustment=((double)adj*time);
        //cout<<current<< "\t" << aim <<"\t "<<adjustment<<endl;
        if (abs(current-aim)<=adjustment*3) return aim;
        else if (current<aim) return (current+adjustment);
        else if (current>aim) return (current-adjustment);

    }
    float main_controller::adjustAng(float current, int max,int min, int adj, double time, int com)
    {

        float adjustment=(adj*time)*com/100;
        float newsp=current+adjustment;
        if(current>min && current <max )return newsp;
        return current;

    }

    class udpBroadcast : public base
    {
    private:
    public:
        void run();
        int sendPacket( char identifier[], char message[], int lenMessage);
        int sendAngle(char identifier[], int angle);
        int sendEE(char identifier[], float EE);
    };

    void udpBroadcast::run()
    {



        while(1)
        {
            sendAngle("A00", getANG(0));
            sendAngle("A01", getANG(1));
            sendAngle("A02", getANG(2));
            sendAngle("A03", getANG(3));
            sendAngle("A04", getANG(4));
            sendAngle("A05", getANG(5));
            sendAngle("A06", getANG(6));
            sendAngle("A07", getANG(7));
            sendAngle("A08", getANG(8));
            sendAngle("A09", getANG(9));
            sendAngle("A10", getANG(10));
            sendAngle("A11", getANG(11));
            sendAngle("A12", getANG(12));
            sendAngle("A13", getANG(13));
            sendAngle("A14", getANG(14));
            sendAngle("A15", getANG(15));

            sendEE("Elx", getEE(0,0));
            sendEE("Ely", getEE(0,1));
            sendEE("Elz", getEE(0,2));
            sendEE("Erx", getEE(1,0));
            sendEE("Ery", getEE(1,1));
            sendEE("Erz", getEE(1,2));



            usleep(10);
        }
    }
    int udpBroadcast::sendPacket( char identifier[], char message[], int lenMessage)
    {
        if (lenMessage>8) return -100;
        char buf[10]= {-1,-1,-1,-1,-1,-1,-1,-1,};
        buf[0]=identifier[0];
        buf[1]=identifier[1];
        buf[2]=identifier[2];
        for(i=0; i<lenMessage; i++) buf[i+3]=message[i];

        return sendto(sockfd, buf, 11, 0,(struct sockaddr*) &cli_addr, slen);

    }

    int udpBroadcast::sendAngle(char identifier[], int angle)
    {
        char message [8]= {0,0,0,0,0,0,0,0};
        if (angle >255)
        {
            message[0]=255;
            message[1]=angle-255;
        }
        else message[0]=angle;

        return sendPacket(identifier, message, 8);

    }
    int udpBroadcast::sendEE(char identifier[], float EE)
    {
        int angMag=abs(EE);
        char message [8]= {0,0,0,0,0,0,0,0};
        if (EE>0) message[0]=1;
        else message[0]=2;

        for(i=1; i<7; i++)
        {
            if(angMag>255)
            {
                message[i]=255;
                angMag-=255;
            }
            else
            {
                message[i]=angMag;
                angMag=0;
            }

        }


        return sendPacket(identifier, message, 8);

    }





    class dataLog : public base
    {
    private:

        ofstream myfile;
    public:
        void run();
        dataLog(char*);
    };

    void dataLog::run()
    {
        ofstream myfile1("Endeffector.txt");

        //myfile << "\t left arm \t \t \t right arm \t \t \t \n";
        //myfile <<"Yaw \t Pitch \t Roll \t Yaw \t Pitch \t Roll \t \n";
        if (myfile1.is_open()) cout<<"file open"<<"\n";
        else cout <<"not open"<<"\n";

        while(1)
        {
            //myfile<< getIMU(3,0)<<"\t " << getIMU(3,1)<<"\t " <<getIMU(3,2)<<" \t ";
            //myfile<< getIMU(1,0)<<"\t " << getIMU(1,1)<<"\t "<< getIMU(1,2)<<" \n ";
            myfile1<< getANG(1)<< "\t" << getANG(5)<< "\t" << getANG(7)<< "\t" << getANG(11)<< "\t" << getEE(1,0)<< "\t" << getEE(1,1)<< "\n" ;

            usleep(10000);
        }
        myfile1.close();
    }
    dataLog::dataLog(char* s):myfile(s)
    {

    }


//_______________________________________________________________________________________
    int main(int argc, char* argv[])
    {



        canInput can;
        boost::thread canThread(boost::bind(&canInput::run, &can));

        humanInput human;
        boost::thread humanThread(boost::bind(&humanInput::run, &human));

        compressor comp;
        boost::thread compThread(boost::bind(&compressor::run, &comp));
//
        information info;
        boost::thread infoThread(boost::bind(&information::run, &info));

        power pow;
        boost::thread powThread(boost::bind(&power::run, &pow));

        balance bal;
        boost::thread balThread(boost::bind(&balance::run, &bal));

        // muscletest muscle;
        //boost::thread muscleThread(boost::bind(&muscletest::run, &muscle));

        kinematics angle;
        boost::thread angleThread(boost::bind(&kinematics::run, &angle));

        main_controller controller;
        boost::thread controllerThread(boost::bind(&main_controller::run, &controller));

        remoteControl motors;
        boost::thread motorsThread(boost::bind(&remoteControl::run, &motors));

        udpBroadcast UDP;
        boost::thread UDPThread(boost::bind(&udpBroadcast::run, &UDP));

        /*PID_CONTROL leftarm(0);
        boost::thread leftarmThread(boost::bind(&PID_CONTROL::run, &leftarm));

        PID_CONTROL rightarm(1);
        boost::thread rightarmThread(boost::bind(&PID_CONTROL::run, &rightarm));

        PID_CONTROL knee(6);
        boost::thread kneeThread(boost::bind(&PID_CONTROL::run, &knee));

        PID_CONTROL hip(7);
        boost::thread hipThread(boost::bind(&PID_CONTROL::run, &hip));

        PID_CONTROL stabaliser(8);
        boost::thread stabaliserThread(boost::bind(&PID_CONTROL::run, &stabaliser));
        */


        //serialOutput serialout(0);
        //boost::thread serialoutThread(boost::bind(&serialOutput::run, &serialout));



        if (argc>1)
        {
            dataLog log(argv[1]);
            boost::thread logThread(boost::bind(&dataLog::run, &log));
        }


        while(1) {}

        return 0;
    }
