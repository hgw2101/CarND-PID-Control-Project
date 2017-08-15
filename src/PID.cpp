#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  /*
  * Errors
  */
  p_error = 0;
  i_error = 0;
  d_error = 0;

  /*
  * Coefficients
  */ 
  Kp = 0;
  Ki = 0;
  Kd = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error; // new cte minus prior cte
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return -(Kp * p_error + Ki * i_error + Kd * d_error);
}