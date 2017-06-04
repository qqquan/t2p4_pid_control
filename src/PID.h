#ifndef PID_H
#define PID_H
#include <vector>


class PID {
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


  std::vector<double> dp;

  int calibr_steps_n;
  double best_error;

  enum{E_HILL_CLIMB_UP,E_HILL_CLIMB_UP_EVALUATION, E_HILL_DESCEND_EVALUATION} calibr_state;

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
  void Init(  double Kp, double Ki, double Kd

          );

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
    return true if a calibration cycle is finished, indicating the caller to reset the simulation
  */
  bool Calibrate(double cte);

  bool isCalibrDone(void);


};

#endif /* PID_H */
