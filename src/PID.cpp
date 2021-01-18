#include "PID.h"
#include <math.h>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  previous_cte=0;  //perhaps change this
  //sum_cte=0;
  total_error=0;
  i_error=0;

  old_t = std::chrono::system_clock::now();

}
void PID::Reset(){
  previous_cte=0;  //perhaps change this
  //sum_cte=0;
  total_error=0;
  i_error=0;

  old_t = std::chrono::system_clock::now();

}

void PID::UpdateError(double cte) {
  
  t = std::chrono::system_clock::now();
  std::chrono::duration<double> dt = t - old_t;

  //mycte= cte;
  p_error=cte;
  d_error=(cte- previous_cte)/dt.count() ;  //supposed to be divided by dt but do we have dt??
  i_error += cte;
  previous_cte = cte;

  old_t = t;
}

// Here we have replaced the original intention of this function. 
//The original functionality is done on GetResult
//This only calculates the error
double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  total_error+= pow(p_error,2);
  return total_error;
//  return 0.0;  // TODO: Add your total error calc here!
}

double PID::GetResult(){

  double res= -Kp * p_error -Kd * d_error -Ki* i_error;
  return  res;
}