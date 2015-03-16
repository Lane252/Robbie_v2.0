#include <iostream>
#include <string>
#include "boost/asio.hpp"
#include <boost/thread/thread.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>11


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

//UDP socket variables

const int PORT = 9930;
struct sockaddr_in my_addr, cli_addr;
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
	    if (className == "canInput") {return true;}
	    else {return false;}
	}

	bool humanInputPer(string className)
	{
	    if (className == "humanInput") {return true;}
	    else {return false;}
	}

	bool balanceInputPer(string className)
	{
	    if (className == "balance") {return true;}
	    else {return false;}
	}

	bool kinematicsInputPer(string className)
	{
	    if (className == "kinematics") {return true;}
	    else {return false;}
	}

	string getClass()
	{
	    return stripNum(typeid(*this).name());
    }

    bool range(int num, int lower, int upper)
    {
        if (num >= lower && num <= upper){return true;}
        else {return false;}
    }

    bool range(int num, int lower)
    {
        if (num >= lower){return true;}
        else {return false;}
    }

	//CAN_IN
    static int PRESSURE[13];
    static int ENCODER[7];
    static int IMU[7][3];
    static int ULTRASONIC[4];
    static int STRINGPOT[2];


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

	virtual ~base(){}

	void sendCan(unsigned char id, unsigned char (&data)[8]);

	bool valid(int num)
    {
        if (num == -1){return false;}
        else {return true;}
    }

    //CAN_IN
    const int& getP(int elem){return PRESSURE[elem];}
	void setP(int num, int elem){if (canInputPer(getClass())) {PRESSURE[elem] = num;}}

    const int& getE(int elem){return ENCODER[elem];}
	void setE(int num, int elem){if (canInputPer(getClass())) {ENCODER[elem] = num;}}

    const int& getIMU(int elem, int pos){return IMU[elem][pos];} //const int (&arr)[7][3] = getIMU();
    void setIMU(int num, int elem, int pos){if (canInputPer(getClass())){IMU[elem][pos] = num;}}

    const int& getU(int elem){return ULTRASONIC[elem];}
	void setU(int num, int elem){if (canInputPer(getClass())) {ULTRASONIC[elem] = num;}}

    const int& getSP(int elem){return STRINGPOT[elem];}
	void setSP(int num, int elem){if (canInputPer(getClass())) {STRINGPOT[elem] = num;}}

	//CAN_OUT

    //PROCESSED
    const int& getANG(int elem){return ANGLE[elem];}
	void setANG(int num, int elem){ANGLE[elem] = num;}


    //APP_IN
	const int& getWL(){return WHEEL_L;}
	void setWL(int num){if (humanInputPer(getClass())) {WHEEL_L = num;}}

	const int& getWR(){return WHEEL_R;}
	void setWR(int num){if (humanInputPer(getClass())) {WHEEL_R = num;}}

	const bool& getCLM(){return CLIMB;}
	void setCLM(bool val){if (humanInputPer(getClass())) {CLIMB = val;}}

	const int& getSTP(){return STEPS;}
	void setSTP(int num){if (humanInputPer(getClass())) {STEPS = num;}}


    //SETPOINTS
    const float& getSETPOINT(int elem){return SETPOINT[elem];}
    void setSETPOINT(float num, int elem){ SETPOINT[elem] = num;}

	//INPUT
    const float& getINPUT(int elem){return INPUT[elem];}
    void setINPUT(float num, int elem){ INPUT[elem] = num;}

	//OUTPUT
    const float& getOUTPUT(int elem){return OUTPUT[elem];}
    void setOUTPUT(float num, int elem) {OUTPUT[elem] = num;}

    //PID
	const double& getPID(int pid, int gain){return PID_GAINS[pid][gain];}
    void setPID(int num, int pid, int gain){PID_GAINS[pid][gain] = num;}

	//COMPRESSOR
    const int& getCOMPRESSOR(){return COMPRESSOR;}
    void setCOMPRESSOR(int num) {COMPRESSOR = num;}

    //OUTPUT
    const int& getCOMMAND(int elem){return COMMAND[elem];}
    void setCOMMAND(int num, int elem) {COMMAND[elem] = num;}


};

//CAN_IN
int base::PRESSURE[13] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
int base::ENCODER[7] = {-1,-1,-1,-1,-1,-1,-1};
int base::IMU[7][3] = {{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1},{-1,-1,-1}};
int base::ULTRASONIC[4] = {-1,-1,-1,-1,};
int base::STRINGPOT[2] = {-1,-1};


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
double base::PID_GAINS[11][3]={{4,0.01,0.0003},{4,0.01,0.0003},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{1,2,0.003},{1,2,0.003},{0,0,0},{0,0,0},{0,0,0}};//gains for PID

//OUTPUT
float base::OUTPUT[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//COMMAND
int base::COMMAND[11] = {0,0,0,0,0,0,0,0,0,0,0};

//COMPRESSOR
int base::COMPRESSOR = 0;

void base::setupUdpSocket()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0)
    {
        err("ERROR opening UDP socket");
    }

    bzero(&my_addr, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(PORT);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int bindReturn = bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr));
    if (bindReturn < 0)
    {
        err("ERROR binding UDP socket");
    }
}

void base::setupCanSocket()
{
    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
	}

	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
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
public:
        void run();
};

void canInput::run()
{
    while(1)
    {
        nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0)
        {
            err("ERROR reading CAN socket");
        }
        switch(frame.can_id)
        {
            //CAN0
            case 0x0B: setE(frame.data[0], 0); break;
            case 0x0C: setIMU(frame.data[0], 0, 0);
                        setIMU(frame.data[1], 0, 1);
                        setIMU(frame.data[2], 0, 2); break;
            case 0x0D: setU(frame.data[0], 0); break;
            case 0x0E: setU(frame.data[0], 1); break;
            case 0x0F: setP(frame.data[0], 0); break;
            case 0x10: setP(frame.data[0], 1); break;

            //CAN1
            case 0x15: setE(frame.data[0], 1); break;
            case 0x16: setIMU(130, 1, 0);
                        setIMU((frame.data[1]), 1, 1);
                        setIMU(130, 1, 2);
                        break;

            case 0x17: setU(frame.data[0], 2); break;
            case 0x18: setU(frame.data[0], 3); break;
            case 0x19: setP(frame.data[0], 2); break;
            case 0x1A: setP(frame.data[0], 3); break;

            //CAN2
            case 0x1F: setIMU(frame.data[0], 2, 0);
                        setIMU((frame.data[1])-72, 2, 1);
                        setIMU(frame.data[2], 2, 2); break;
            case 0x1C: setP(frame.data[0], 4); break;
            case 0x1D: setP(frame.data[0], 5); break;

            //CAN3
            case 0x29: setP(frame.data[0], 6); break;
            case 0x2A: setP(frame.data[0], 7); break;
            case 0x2B: setIMU(1, 3, 0);
                        setIMU(frame.data[2]+frame.data[3], 3, 1);break;
                        setIMU(3, 3, 2);

            //CAN4
            case 0x33: setE(frame.data[0], 2); break;
            case 0x34: setIMU(frame.data[0]+frame.data[1], 4, 0);
                        setIMU((frame.data[1]), 4, 1);
                        setIMU(frame.data[2], 4, 2); break;
            case 0x36: setP(frame.data[0], 8); break;



            //CAN5
            case 0x3D: setE(frame.data[0], 3); break;
            case 0x3E: setE(frame.data[0], 4); break;
            case 0x3F: setIMU(frame.data[0], 5, 0);
                        setIMU((frame.data[1])-77, 5, 1);
                        setIMU(frame.data[2], 5, 2); break;

            //CAN6
            case 0x47: setE(frame.data[0], 5); break;
            case 0x48: setSP(frame.data[0], 0); break;
            case 0x49: setP(frame.data[0], 9); break;
            case 0x4A: setP(frame.data[0], 10); break;
            case 0x4B: setIMU(frame.data[0], 6, 0);
                        setIMU((frame.data[1])-165, 6, 1);
                        setIMU(frame.data[2], 6, 2); break;

            //CAN7
            case 0x51: setE(frame.data[0], 6); break;
            case 0x52: setSP(frame.data[0], 1); break;
            case 0x53: setP(frame.data[0], 11); break;
            case 0x54: setP(frame.data[0], 12); break;
            case 0x55: setIMU(frame.data[0], 7, 0);
                        setIMU(frame.data[1], 7, 1);
                        setIMU(frame.data[2], 7, 2); break;

        }
    }
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
    close(sockfd);
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

        angDeg(getANG(2), "STAB_SHANK ");
        angDeg(getANG(4), "KNEE-SHANK ");
        angDeg(getANG(8), "KNEE-UPPER ");
        angDeg(getANG(9), "RIGHT_ARM ");
        angDeg(getANG(10), "RIGHT_ARM-UPPER ");
        angDeg(getANG(11), "LEFT_ARM ");
        angDeg(getANG(12), "LEFT_ARM-UPPER ");
        psiBar(getP(5), "Tank Pressure");

        pidVal(getINPUT(1), getSETPOINT(1), getOUTPUT(1), "Right Arm PID");
        pidVal(getINPUT(0), getSETPOINT(0), getOUTPUT(0), "Left Arm PID");

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
    {	double p=getPID(pid,0);
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
private:
const static int RANGE=360;
const static int def_STAB=18;
const static int def_SHANK=163;
const static int def_KNEE=20;
const static int def_UPPER=117;
const static int def_ARM=270;
const static int def_alpha=148;
public:

        void run();
};

void kinematics::run()
{unsigned long start=(std::clock()/((double) CLOCKS_PER_SEC));



    while(1)
    {
        //Stabiliser angle
        setANG(180+def_STAB-(getIMU(6,1)),0);

        //SHANK
        setANG(180+def_SHANK+ getIMU(5,1),1);

        //Stabiliser-Shank angle

        setANG(getANG(1)-getANG(0),2);

        //Init Knee Angle
        setANG((getIMU(4,0)),3);

        //SHANK-THIGH ANGLE

        setANG(180-def_SHANK-getIMU(5,1)+getANG(3),4);

        //KNEE-UPPERBODY ANGLE

        setANG(def_UPPER+ getIMU(2,1),5);

        setANG(180-getANG(5),6);

        setANG(getANG(3),7);

        setANG(getANG(6)+getANG(7),8);


        //ARM ANGLES

            //RIGHT
        setANG(def_ARM -getIMU(1,1),9);
        setANG((getANG(9)+getIMU(2,1)),10);
            //LEFT
        setANG(def_ARM -getIMU(3,1),11);
        setANG((getANG(11)+getIMU(2,1)),12);





        usleep(10000);
    }
}
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
 unsigned char serialOutput::messageone[5]={255, 127,127,127,127};
 unsigned char serialOutput::messagetwo[4]={127,127, 127,127};
 unsigned char serialOutput::messagethree[4]={127,127,127,127};



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
		 sleep(0.01);


		try
		{
            //boost::asio::write(serial,boost::asio::buffer(&message,length));
            boost::asio::write(serial,boost::asio::buffer(&messageone,5));
            boost::asio::write(serial,boost::asio::buffer(&messagetwo,4));
            boost::asio::write(serial,boost::asio::buffer(&messagethree,4));
            i=i+1;
            //cout <<i<<"\t"<< j<<endl;

		}catch(boost::system::system_error& e)
		{ j =j+1;
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
{   wheel_drive=1;
int adj=50;
int max=100;
    while(1)
    {



        c=getche();


        switch(c)
        {   case ' ':
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
                {cout <<"ground wheels"<< endl;
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
    for(int i=0;i<11;i++)
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
{   //com has range -100 to 100. -100 means move to minimum at fastest rate, 100 means move to max.
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


myfile << "\t left arm \t \t \t right arm \t \t \t \n";
myfile <<"Yaw \t Pitch \t Roll \t Yaw \t Pitch \t Roll \t \n";
    while(1)
    {    myfile<< getIMU(3,0)<<"\t " << getIMU(3,1)<<"\t " <<getIMU(3,2)<<" \t ";
        myfile<< getIMU(1,0)<<"\t " << getIMU(1,1)<<"\t "<< getIMU(1,2)<<" \n ";


        sleep(0.1);
    }
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


	serialOutput serialout(0);
    boost::thread serialoutThread(boost::bind(&serialOutput::run, &serialout));



    if (argc>1)
    {
        dataLog log(argv[1]);
        boost::thread logThread(boost::bind(&dataLog::run, &log));
    }


	while(1){}

    return 0;
}
