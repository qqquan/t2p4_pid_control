#include "PID.h"
#include <iostream> 
// using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
}

void PID::UpdateError(double cte) {
    p_error = Kp*cte;
    i_error = Ki*cte;
    d_error = Kd*cte;

    std::cout <<  "p: "<<p_error<<"; i: "<<i_error<<"; d: "<< d_error << std::endl;
}

double PID::TotalError() {
    return p_error + i_error + d_error;
}             

