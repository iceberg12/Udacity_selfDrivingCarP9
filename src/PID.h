#ifndef PID_H
#define PID_H

#include <vector>
using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error, i_error, d_error;

  /*
  * Coefficients
  */ 
  double Kp, Ki, Kd;
  
  // Twiddle
  std::vector<double> dp;
  int step, param_idx;
  
  int settle_steps, eval_steps;
  bool PID_add;
  
  // Other errors
  double total_error, best_error;
  
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte_);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  * Adjust PID parameters by index
  */
  void AdjPID(int index, double x);
};

#endif /* PID_H */
