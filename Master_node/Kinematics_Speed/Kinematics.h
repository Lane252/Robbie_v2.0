#include "Eigen/Eigen"
#include <cstdio>
#include <ctime>
#include <iostream>
#include <vector>
#include <fstream>

#define JOINTS 3
#define TOLERANCE 0.001
#define MAX_ITER 50000
#define EF_ERROR_TOLERANCE 0.001


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix4d;

class kinematics
{
Eigen::VectorXd Theta;
Eigen::VectorXd Alpha;
Eigen::VectorXd d;
Eigen::VectorXd a;
Eigen::VectorXd ef_g;
Eigen::VectorXd s;
Eigen::MatrixXd p;
Eigen::MatrixXd Theta_Lim;
const static float RAD= M_PI/180;//Radian Conversion
const static float DEG=180/M_PI;


private:
//DH Parameters
    //---------------------------------Theta-------Alpha-----------d-----------a-----
    /*JointBase*/const static float  Th_B=90,     Alp_B=0,       d_B=0,      a_B=0;
    /*Joint 1*/const static float    Th_1=90,    Alp_1=0,       d_1=0,      a_1=2;
    /*Joint 2*/const static float    Th_2=40,     Alp_2=0,       d_2=0,      a_2=1;
    /*Joint 3*/const static float    Th_3=20,    Alp_3=0,       d_3=0,      a_3=1;
    //*Joint 3*/float    Th_4=0.87,    Alp_4=0,       d_4=0,      a_4=1;

    //Joint Limits
    const static float Th_B_Max=90,Th_B_Min=90;
    const static float Th_1_MAX=40, Th_1_MIN=330;
    const static float Th_2_MAX=50,Th_2_MIN=200;
    const static float Th_3_MAX=40,Th_3_MIN=240;

    const static float EE_x=2.2, EE_y=2, EE_z=0;


void DHMatrix(MatrixXd&T, double alpha, double a, double d, double theta);
void FKine(MatrixXd &p, VectorXd &s,VectorXd theta, VectorXd alpha, VectorXd a, VectorXd d);
void findJ(MatrixXd &J, MatrixXd s, MatrixXd p);
void findTranspose(MatrixXd &J_I, MatrixXd &J);
void findPinv(MatrixXd &J_I, MatrixXd &J);
void DLS_Method(MatrixXd &J_I, MatrixXd &J);
void findDeltaTheta(VectorXd &delta_theta, MatrixXd &J, MatrixXd &J_I, VectorXd ef_g, VectorXd ef_c);
void JacobianIK(VectorXd ef_g, VectorXd &theta, VectorXd alpha, VectorXd a,VectorXd d, MatrixXd &theta_lim);
void JointClamp(VectorXd H,MatrixXd &H_M, MatrixXd &theta_lim, VectorXd &theta, VectorXd &delta_theta);
void Grad_Proj(VectorXd H,MatrixXd &H_M, MatrixXd &theta_lim, VectorXd &theta, VectorXd &delta_theta);
void Check_Goal(VectorXd &ee_g, VectorXd &a);
void Check_EE(VectorXd &s);



public:
kinematics();
~kinematics();
void run();
};
