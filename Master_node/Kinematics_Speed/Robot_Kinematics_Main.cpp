#include "Kinematics.h"
#include <iostream>
 using namespace std;
int main(){
    ofstream Time;
    Time.open("Transpose_Time.txt",fstream::app);

kinematics Robot;
std::clock_t start;
double duration;
start=std::clock();
Robot.run();
duration= ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
std::cout<<"printf: "<< duration <<'\n';
Time<<duration<<endl;
Time.close();

while(1){}
}
