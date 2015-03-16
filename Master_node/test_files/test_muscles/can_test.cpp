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



// wifi password on robot: C1FBBB2E

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
    static int SETPOINT[8];// LEFT arm, RIGHT arm...

    //PID
    static double PID_GAINS[8][3];//2 PIDs, 3 values for each

	//PID INPUTS
	static int INPUT[8];

	//PID OUTPUTS
	static int OUTPUT[20];

    //PROCESSED
    static int ANGLE[15];

	//COMPRESSOR
	static int COMPRESSOR;

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
    const int& getSETPOINT(int elem){return SETPOINT[elem];}
    void setSETPOINT(int num, int elem){ SETPOINT[elem] = num;}

	//INPUT
    const int& getINPUT(int elem){return INPUT[elem];}
    void setINPUT(int num, int elem){ INPUT[elem] = num;}

	//OUTPUT
    const int& getOUTPUT(int elem){return OUTPUT[elem];}
    void setOUTPUT(int num, int elem) {OUTPUT[elem] = num;}

    //PID
	const double& getPID(int pid, int gain){return PID_GAINS[pid][gain];}
    void setPID(int num, int pid, int gain){PID_GAINS[pid][gain] = num;}

	//COMPRESSOR
    const int& getCOMPRESSOR(){return COMPRESSOR;}
    void setCOMPRESSOR(int num) {COMPRESSOR = num;}


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
int base::SETPOINT[8] = {-1,-1,-1,-1,-1,-1,-1,-1};

//INPUT
int base::INPUT[8] = {-1,-1,-1,-1,-1,-1,-1,-1};

//PID
double base::PID_GAINS[8][3]={{4,0.01,0.0003},{4,0.01,0.0003},{1,2,0.003},{1,2,0.003},{1,2,0.003},{1,2,0.003},{1,2,0.003},{1,2,0.003}};//gains for PID

//OUTPUT
int base::OUTPUT[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

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
            case 0x1B: setIMU(130, 1, 0);
                        setIMU((frame.data[2] + frame.data[3]), 1, 1);
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
            case 0x34: setIMU(frame.data[0], 4, 0);
                        setIMU((frame.data[1])-110, 4, 1);
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
                setCOMPRESSOR(128);
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

        sleep(0.01);
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
        sleep(0.01);
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
        setANG(-(getIMU(4,1))+def_KNEE,3);

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




        setINPUT(getANG(10),1);//right arm
        setINPUT(getANG(11),0);//left arm
        setINPUT(getANG(5),7);//hip
        if (((std::clock()/((double) CLOCKS_PER_SEC))-start)>10)
        {
            //setSETPOINT(250,1);
        }


        sleep(0.01);
    }
}
//_______________________________________________________________________________________


//,
class serialOutput : public base
{
private:
        static const unsigned int length =3;
        static unsigned char message[length];

        boost::asio::io_service io;
        boost::asio::serial_port serial;

public:

        void run();
        serialOutput(int a): io(), serial(io,"/dev/ttyO4")// pin 13 is tx
    {


        serial.set_option(boost::asio::serial_port_base::baud_rate(19200));
    }

};
 unsigned char serialOutput::message[length]={255, 127,127};



void serialOutput::run()
{
int i=0;
int j=0;

    while(1)
    {
        //MOTORS
		message[1]=getCOMPRESSOR()+127;//compressor
		//message[2]=getOUTPUT(10); //valve 3
		//message[3]=getOUTPUT(11); //valve 4
        message[2]=getOUTPUT(9); //valve 3
        int a=message[2];


		/*message[2]=getOUTPUT(0)+127;//left arm
		message[3]=getOUTPUT(1)+127;//right arm
		message[4]=getOUTPUT(2)+127;//ground wheels left
		message[5]=getOUTPUT(3)+127;//ground wheel right
		message[6]=getOUTPUT(4)+127;//stair wheel left
		/*message[7]=getOUTPUT(5)+127;//stair wheel right
		message[8]=getOUTPUT(6)+127;//knee
		message[9]=getOUTPUT(7)+127;//hip
        */
        //message[7]=getOUTPUT(8);
		//SOLENOID VALVES
       // message[10]=getOUTPUT(19)+127;//compressor

		/*message[7]=getOUTPUT(8);
        message[8]=getOUTPUT(9);
		message[9]=getOUTPUT(10);
		message[10]=getOUTPUT(11);
		message[11]=getOUTPUT(12);
		message[12]=getOUTPUT(13);


		/*
		message[10]=getOUTPUT(11);
		message[3]=getOUTPUT(12);
		message[4]=getOUTPUT(13);
		message[5]=getOUTPUT(14);
		message[6]=getOUTPUT(15);

        //message[7]=getCOMPRESSOR()+127;

		/*message[7]=getOUTPUT(16);
		message[8]=getOUTPUT(17);
		message[9]=getOUTPUT(18);
		//message[10]=getOUTPUT(19);
        message[11]=getOUTPUT(2)+127;//ground wheels left
		message[12]=getOUTPUT(3)+127;//ground wheel right*/
		//boost::asio::write(serial,boost::asio::buffer(message,length));
		 sleep(0.01);
		//char s[7]={0X01, 0X02, 0X03,0X04, 0X05, 0X06,0X07};

		try
		{
            boost::asio::write(serial,boost::asio::buffer(&message,length));
            i=i+1;
            //cout <<i<<"\t"<< j<<endl;

		}catch(boost::system::system_error& e)
		{ j =j+1;
		    cout<<"serial send did not work "<<j<<endl;
		    //return ;
		}



        sleep(0.01);
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

char c;
public:
        void run();
};

void remoteControl::run()
{
    while(1)
    {
        c=getche();
        unsigned int zero='0';
        unsigned int num=c-zero;

        if (num>=0 && num<10)
        {
           setOUTPUT(num,9);
        }


        setOUTPUT((c-zero),9);

        switch(c)
        {case 'm':
            cout<<"\nkill\n";
            setCOMPRESSOR(0);
            sleep(2);
            return;
            break;

            case 'i':
            setOUTPUT((10),9);
            break;

            case 'o':
            setOUTPUT((11),9);
            break;

            case 'p':
            setOUTPUT((12),9);
            break;
        }

/*
            case 'w':
            cout<<"forward"<<endl;
            setOUTPUT(127,2);
            setOUTPUT(127,3);
            break;

            case 'a':
            cout<<"left turn"<<endl;
            setOUTPUT(0,2);
            setOUTPUT(127,3);
            break;

             case 's':
             cout<<"reverse"<<endl;
            setOUTPUT(-127,2);
            setOUTPUT(-127,3);
            break;

             case 'd':
             cout<<"turn right"<<endl;
            setOUTPUT(127,2);
            setOUTPUT(0,3);
            break;

             case 'q':
             cout<<"drift left"<<endl;
            setOUTPUT(60,2);
            setOUTPUT(127,3);
            break;

             case 'e':
             cout<<"drift right"<<endl;
            setOUTPUT(127,2);
            setOUTPUT(60,3);
            break;

            case 'x':
             cout<<"stop"<<endl;
            setOUTPUT(0,2);
            setOUTPUT(0,3);
            break;

            //Muscle Control
            case '1':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,8);break;
            case '2':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,9);break;

            case '3':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,10);break;

            case '4':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,11);break;

            case '5':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,12);break;

            case '6':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,13);break;

            case '7':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,14);break;

            case '8':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,15);break;

            case '9':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,16);break;

            case 'i':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,17);break;

            case 'o':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,18);break;

            case 'c':
            setOUTPUT(127,19);
            break;
            /*case 'p':
            cout<<"\ncase0\n";
            for (i=8; i<20;i++){setOUTPUT(0,i);}
            setOUTPUT(1,19);break;
            */
          /*  case '0':
            cout<<"\ncase0\n";
            for (i=0; i<20;i++){setOUTPUT(0,i);
             }
            break;

            case '.':
            cout<<"\ncase0\n";
            for (i=0; i<20;i++){setOUTPUT(1,i);
            setCOMPRESSOR(127);}
            break;

            case 'm':
            cout<<"\nkill\n";
            setCOMPRESSOR(0);
            sleep(1);
            return;
            break;



        }

*/

        sleep(0.1);
    }
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
   // boost::thread muscleThread(boost::bind(&muscletest::run, &muscle));

    kinematics angle;
    boost::thread angleThread(boost::bind(&kinematics::run, &angle));

    remoteControl motors;
    boost::thread motorsThread(boost::bind(&remoteControl::run, &motors));

    PID_CONTROL leftarm(0);
    boost::thread leftarmThread(boost::bind(&PID_CONTROL::run, &leftarm));

    PID_CONTROL rightarm(1);
    boost::thread rightarmThread(boost::bind(&PID_CONTROL::run, &rightarm));

    // PID_CONTROL hip(7);
    //boost::thread rightarmThread(boost::bind(&PID_CONTROL::run, &rightarm));


    //canOutput canout;
    //boost::thread canoutThread(boost::bind(&canOutput::run, &canout));

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
