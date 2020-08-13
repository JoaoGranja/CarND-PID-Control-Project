#ifndef PID_H
#define PID_H

#include <vector>
#include <cmath>
#include <iostream>
#include <limits>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);
  

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  /**
   * Update PID control gains based on cte error
   */
  void Twiddle(double cte);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  std::vector<double> KG;
  
  // variables for twiddle algorithm
  double best_error;
  int twiddle_indice;
  int twiddle_state;
  std::vector<double> dp;
  int twiddle_count = 0;
};

#endif  // PID_H