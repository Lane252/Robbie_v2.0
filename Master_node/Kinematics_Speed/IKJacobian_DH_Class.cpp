
#include "Kinematics.h"

using namespace std;

//Jacobian PseudoInverse Inverse Kinematics
//Author: George Walsh MAI
//Date:21/1/15

//This file contains the functions:
//DHMatrix:         Takes in the DH parameters for the robotic kinematic chain
//FKine:            Solves the Forward Kinematics for the robotic kinematic chain by
//                  multiplying the homogeonous DH matrices together and finding the
//                  end effector position in cartesian (x,y,z) space.
//
//FindJ:            Calculates the Jacobian Matrix.
//FindPinv:         Calculates the PseudoInverse Jacobain Matrix
//FindDeltaTheta:   Calculates the change in joint angles and updates them in the main inverse
//                  kinematics function.
//JacobianPINV_IK:  This is the main inverse kinematics function. This function take in the current position of
//                  the end effector, the current joint values and all of the DH parameters. It then uses an
//                  iterative method to calculate the goal end effector position. The joint variables required to
//                  achieve this configuration are then output.
//
//JointClamp:       This clamps the joints in place when they reach a limit. Uses a boolean function to determine when joint has
//                  reached the limit. Not the most elegant solution, but does the job.
//
//The Eigen library is used for the matrix manipulation.

//Jacobian Algorithms: A Robotics Toolbox, Author: P.Corke &&
//                     Introduction to Inverse Kinematics with Jacobian Transpose , Pseudoinverse
//                     and Damped Least Squares methods, Author: S.Buss



kinematics::kinematics(){
//cout<<"Here"<<endl;
}
kinematics::~kinematics(){}
void kinematics::DHMatrix(MatrixXd&T, double alpha, double a, double d, double theta)
{
//cout<<"DH"<<endl;
T.setIdentity();
//DH Matrix Init
T(0,0)=cos(theta);  T(0,1)=-cos(alpha)*sin(theta);  T(0,2)=sin(alpha)*sin(theta);   T(0,3)=a*cos(theta);
T(1,0)=sin(theta);  T(1,1)=cos(theta)*cos(alpha);   T(1,2)=-cos(theta)*sin(alpha);  T(1,3)=a*sin(theta);
T(2,0)=0;           T(2,1)=sin(alpha);              T(2,2)=cos(alpha);              T(2,3)=d;
T(3,0)=0;           T(3,1)=0;                       T(3,2)=0;                       T(3,3)=1;

}

void kinematics::FKine(MatrixXd &p, VectorXd &s,VectorXd theta, VectorXd alpha, VectorXd a, VectorXd d){
    vector<MatrixXd> T;
//cout<<"Fkine"<<endl;
    MatrixXd TTCurr(4,4); //Current Homogenous Transformation Matrix
    MatrixXd TTPrev(4,4); //Previous Homogenous Transformation Matrix


    TTCurr.setIdentity();

    TTPrev.setIdentity();


    for (int i = 0; i <JOINTS; i++) {

    DHMatrix(TTCurr,  alpha[i], a[i], d[i],theta[i]);

    TTCurr=TTPrev*TTCurr;

    TTPrev=TTCurr;
//cout<<"Here1"<<endl;


//Current Joint Angles
    p(0, i) = TTPrev(0,3);

    p(1, i) = TTPrev(1,3);

    p(2, i) = TTPrev(2,3);


//cout<<"Here2"<<endl;

    }
//Current End Effector Position
    s(0) = TTPrev(0,3);

    s(1) = TTPrev(1,3);

    s(2) = TTPrev(2,3);

   // s(3) = TTPrev(3,3);

   //***** Needs changing depending on DOF

   //cout<<s<<endl;
}

void kinematics::findJ(MatrixXd &J, MatrixXd s, MatrixXd p)
{
//cout<<"FindJ"<<endl;
    for (int j = 0; j < J.cols(); j++) {
        Eigen::Vector3d d = s.col(0) - p.col(j);
        J(0,j) = d(2) - d(1) ;
        J(1,j) = d(0) - d(2) ;
        J(2,j) = d(1) - d(0) ;

    }
}
void kinematics::findTranspose(MatrixXd &J_I, MatrixXd &J)
{
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

void kinematics::DLS_Method(MatrixXd &J_I, MatrixXd &J)
{
    MatrixXd I(3,3);
    I << 100,100,100,
         100,100,100,
         100,100,100;

    MatrixXd inver = J*J.transpose() + I;
    J_I = J.transpose() * inver.inverse();
}
void kinematics::findDeltaTheta(VectorXd &delta_theta, MatrixXd &J, MatrixXd &J_I, VectorXd ef_g, VectorXd ef_c){
//cout<<"Delta Theta"<<endl;
    findTranspose(J_I, J);
     //findPinv(J_I, J);
    //DLS_Method(J_I,J);
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
    ofstream error;
    ofstream Itter;

    Theta.open ("_Theta.txt");

    error.open("Transpose_Error.txt",fstream::app);
    Itter.open("Transpose_Itter.txt",fstream::app);

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
    VectorXd Error(3);


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

        theta = theta + delta_theta; // update value of theta
       JointClamp(H,H_M,theta_lim,theta,delta_theta);

       //Grad_Proj(H,H_M,theta_lim,theta,delta_theta);
//        cout<<"Here7"<<endl;
        Theta<<iter<<"\t"<< theta(0)*DEG<<"\t "<<theta(1)*DEG<<"\t"<<theta(2)*DEG<<endl;

        FKine(p, s, theta, alpha, a,d); // update v

        diff_ee = s - ef_g;

        iter++;

       // cout<<"iteration: "<<iter<<endl;
        //cout << "End Effector Position: " << endl << s << endl;

        if(iter>=MAX_ITER){

            cout<<"Iteration Limit Reached: "<<iter<<endl;
            cout<<"Could not find Valid Solution"<<endl;
            cout << "Final End Effector Position: " << endl << s << endl;
            std::cout << "Joint Angles:" <<endl<< theta << endl;
            return;
        }
    }


    for(int i=0;i<3;i++){
    Error(i)=abs((s(i))-ef_g(i));
    }
    Itter<<iter<<endl;
    error<<Error(0)<<"\t"<<Error(1)<<"\t"<<Error(2)<<"\t"<<endl;

    Itter.close();
    error.close();
    Theta.close();

   // Check_EE(s);
    cout << "Final End Effector Position: " << endl << s << endl;
    std::cout << "Joint Angles:" <<endl<< theta << endl;
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


void kinematics::Grad_Proj(VectorXd H,MatrixXd &H_M, MatrixXd &theta_lim, VectorXd &theta, VectorXd &delta_theta){
//cout<<"Grad_Proj"<<endl;
 for(int i=0;i<JOINTS;i++){

    H(i)=(0.25)*((pow((theta_lim(i,0)-theta_lim(i,1)),2))/((theta_lim(i,0)-theta(i))*(theta(i)-theta_lim(i,1))));
 }

H_M<<H(0),0,0,
     0,H(1),0,
     0,0,H(2);
     //0,0,0,H(3);


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
    return;
}
cout<<"Target in reach"<<endl;


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


