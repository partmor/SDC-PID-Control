#ifndef PID_H
#define PID_H

class PID {

  // attribute to store current cross track error
  double prev_cte;

  // flag only active for first step (related to derivative initialization)
  bool first_step;

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Output control command.
  */
  double OutputControl();
};

#endif /* PID_H */
