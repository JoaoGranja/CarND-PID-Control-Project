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
  static int error_count = 0;
  std::vector<double> p = {Kp, Ki, Kd};
  double error, average_error;

  sum_of_elems = dp[0] + dp[1] + dp[2];
  
  // if sum of dp elements is too small, return
  if(sum_of_elems < 0.0001){
    return;
  }
  
  // As the cte changes during the simulation, I consider the average error instead of minimum one
  if(error_count <10)
  {
    n_errors.push_back(p_error*p_error);
    indice = 0;
    twiddle_state = 0;
    error_count ++;
    return;
  }
  
  average_error = accumulate( n_errors.begin(), n_errors.end(), 0.0)/ n_errors.size();
  
  error = p_error*p_error;
  
  //Remove last error and add a new one
  n_errors.pop_back();
  n_errors.push_back(error);

  std::cout << "error: " << error << " average_error " << average_error << std::endl;
  
  switch(twiddle_state)
  {
    case 0:
      // increase the control gain
 	  twiddle_state = 1;
      p[indice] += dp[indice];
      break;
    case 1:
      if(error < average_error){
        // increase the delta control gain
        std::cout << "increase the delta control gain " << indice << std::endl;
        dp[indice] *= 1.1;
        twiddle_state = 0;
        indice = (indice + 1) % 3;
      }
      else
      {
        // decrease the control gain
        p[indice] -= 2 * dp[indice];
        twiddle_state = 2;
      }
      break;
    case 2:
      if(error < average_error){
        // increase the delta control gain
        std::cout << "increase the delta control gain " << indice << std::endl;
        dp[indice] *= 1.1;
      }
      else
      {
        // decrease the delta control gain
        p[indice] += dp[indice];
        std::cout << "decrease the delta control gain " << indice << std::endl;
        dp[indice] *= 0.9;
      }
      twiddle_state = 0;
      indice = (indice + 1) % 3;
      break;
    default:
      break;
  } 

  UpdateGain(p[indice], indice);
  std::cout << "Kd = " << Kd << "   Ki = " << Ki  << "   Kp = " << Kp << std::endl;
}
  
  