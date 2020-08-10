#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

#define INIT           (int)0    // Initial state of Twiddle
#define INCREASE_DELTA (int)1    // Increase the delta gain
#define DECREASE_DELTA (int)2    // Decrease the delta gain

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  KG = {Kp_, Ki_, Kd_};
  
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  dp = {0.1*Kp_, 0.1*Ki_, 0.1*Kd_};
  
  twiddle_indice = 0;
  twiddle_state = INIT;
  best_error = 100000;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error;
  total_error = -KG[0] * p_error - KG[1] * i_error - KG[2] * d_error;
  
  //remember the steering value should be [-1, 1].
  if(total_error > 1.0)
    return 1.0;
    
  if(total_error < -1.0)
    return-1.0;
    
  return total_error;  // TODO: Add your total error calc here!
}

void PID::Twiddle(double cte, double steer_error){
  
  double sum_of_elems = 0.0;
  sum_of_elems = dp[0] + dp[1] + dp[2];
  double error;
  
  // if sum of dp elements is too small, return
  if((sum_of_elems < 0.0001))
  {
    std::cout << "Kp = " << KG[0] << "   Ki = " << KG[1]  << "   Kd = " << KG[2] << std::endl;
    twiddle_count ++;
    return;
  }
  
  std::cout << "cte: " << cte  << " best_error " << best_error << std::endl;
  
  if(cte < 0)
    cte *= -1.0;
  
  if(steer_error < 0)
    steer_error *= -1.0;
  //if((twiddle_count % 100 ) == 0)
  //  best_error = 100000;
  
  error = cte + steer_error;
  
  switch(twiddle_state)
  {
    case INIT:
      // increase the control gain
 	  twiddle_state = INCREASE_DELTA;
      KG[twiddle_indice] += dp[twiddle_indice];
      std::cout << "increase the delta gain " << twiddle_indice << std::endl;
      break;
    case INCREASE_DELTA:
      if(error <best_error){
        // increase the delta control gain
        std::cout << "increase the delta control gain " << twiddle_indice << std::endl;
        dp[twiddle_indice] *= 1.1;
        twiddle_state = INIT;
        twiddle_indice = (twiddle_indice + 1) % 3;
        best_error = error;
      }
      else
      {
        // decrease the control gain
        KG[twiddle_indice] -= 2 * dp[twiddle_indice];
        twiddle_state = DECREASE_DELTA;
        std::cout << "decrease the delta gain " << twiddle_indice << std::endl;
      }
      break;
    case DECREASE_DELTA:
      if(error < best_error){
        // increase the delta control gain
        std::cout << "increase the delta control gain " << twiddle_indice << std::endl;
        dp[twiddle_indice] *= 1.1;
        best_error = error;
      }
      else
      {
        // decrease the delta control gain
        KG[twiddle_indice] += dp[twiddle_indice];
        std::cout << "decrease the delta control gain " << twiddle_indice << std::endl;
        dp[twiddle_indice] *= 0.9;
      }
      twiddle_state = INIT;
      twiddle_indice = (twiddle_indice + 1) % 3;
      break;
    default:
      break;
  } 

  std::cout << "Kp = " << KG[0] << "   Ki = " << KG[1]  << "   Kd = " << KG[2] << std::endl;
  twiddle_count ++;
  return;
}
  
  