#include "PID.h"

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  // set PID coefficients
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // prepare controller for its first step
  first_step = true;

  // set initial errors to zero
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte) {

  // placeholder for time step
  double dt = 1.0;

  // proportional error
  p_error = cte;

  // intergral error
  i_error += cte * dt;

  // derivative error
  // during first step there is not enough available information to
  // calculate a precise derivative
  if (first_step){
    first_step = false;
  } else{
    d_error = (cte - prev_cte) / dt;
  }

  // update previous cte
  prev_cte = cte;

}

double PID::OutputControl() {
  return - Kp * p_error - Ki * i_error - Kd * d_error;
}

