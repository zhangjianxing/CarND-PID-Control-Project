#include <iostream>
#include <math.h>
#include "PID.h"


using namespace std;

PID::PID() {}
PID::~PID() {}


void PID::Init(double Kp, double Kd, double Ki) {
    /*Set Kp, Ki and Kd to initial values passed by controller. These are passed from main*/
    double src[] = {Kp, Kd, Ki};
    vector<double> p;
    p.assign(std::begin(src), std::end(src));
    this->SetP(p);
}

void PID::SetP(vector<double> p) {
    /*Set Kp, Ki and Kd to initial values passed by controller. These are passed from main*/
    this->p.assign(std::begin(p), std::end(p));
    
    /*Set up inital p, i and d error to zero.*/
    this->p_error = 0;
    this->i_error = 0;
    this->d_error = 0;
    twiddle_error = 0;
}

vector<double> PID::GetP() {
    vector<double> _p;
    _p.assign(std::begin(p), std::end(p));
    return _p;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    twiddle_error += cte*cte;
}

double PID::TotalError() {
    double total_error = -(p[0] * p_error + p[1] * d_error + p[2] * i_error);
    if (total_error < -1) {return -1;}
    if (total_error > 1) {return 11;}
    return total_error;
}

double PID::get_twiddle_error() {
    return this->twiddle_error;
}
