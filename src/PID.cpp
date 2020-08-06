#include "PID.h"

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
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

void PID::UpdateGain(double gain, int indice) {
  /**
   * TODO: Update PID gain.
   */
  if(indice == 0)
  {
    Kp = gain;
  }
  else if(indice == 1)
  {
  	Ki = gain;
  }
  else{
  	Kd = gain;
  }
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error;
  total_error = -Kp * p_error - Kd * d_error - Ki * i_error;
  return total_error;  // TODO: Add your total error calc here!
}

void PID::Twiddle(){
  
  double sum_of_elems = 0.0;
  static bool first_time = true;
  std::vector<double> p = {Kp, Ki, Kd};
  double error;

  sum_of_elems = dp[0] + dp[1] + dp[2];
  
  // if sum of dp elements is too small, return
  if(sum_of_elems < 0.0001){
    return;
  }
  
  if(first_time)
  {
    best_error = 100000;
    indice = 0;
    first_time = false;
  }
  
  
  // increase the control gain
  p[indice] += dp[indice];
  UpdateGain(p[indice], indice);
  error = TotalError();

  if(error < best_error){
    best_error = error;
    std::cout << "increase the control gain " << indice << std::endl;
    dp[indice] *= 1.1;
  }
  else
  {
    // decrease the control gain
    p[indice] -= 2 * dp[indice];
    UpdateGain(p[indice], indice);
    error = TotalError();

    if(error < best_error){
      best_error = error;
      std::cout << "decrease the control gain " << indice << std::endl;
      dp[indice] *= 1.1;
    }
    else
    {
      // decrease the delta control gain
      p[indice] += dp[indice];
      std::cout << "decrease the delta control gain " << indice << std::endl;
      dp[indice] *= 0.9;
    }
  }   

  UpdateGain(p[indice], indice);
  indice = (indice + 1) % 3;
}
  
  