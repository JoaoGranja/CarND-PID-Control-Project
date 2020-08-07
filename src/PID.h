#ifndef PID_H
#define PID_H

#include <vector>
#include <numeric>
#include <iostream>

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
   * Update the PID control gain.
   * @param gain new control gain
   * @param indice indice of the new control gain
   */
  void UpdateGain(double gain, int indice);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  /**
   * Update PID control gains based on total PID error
   */
  void Twiddle();

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
  
  std::vector<double> n_errors;
  int indice;
  int twiddle_state;
  std::vector<double> dp = {0.001, 0.001, 0.001};
};

#endif  // PID_H