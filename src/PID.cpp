#include "PID.h"
#include <iostream> 
#include <numeric>


// using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    PID:dp = {1,1,1};
    PID::calibr_steps_n = 300;
    PID::calibr_state = E_HILL_CLIMB_UP;
    PID::best_error = 0;
}

void PID::UpdateError(double cte) {
    p_error = Kp*cte;
    i_error = Ki*cte;
    d_error = Kd*cte;

    std::cout <<  "p: "<<p_error<<"; i: "<<i_error<<"; d: "<< d_error << std::endl;
}

double PID::TotalError() {
    return p_error + i_error + d_error;
}             

bool PID::isCalibrDone(void)
{
    double sum_of_dp = std::accumulate(dp.begin(), dp.end(), 0);

    return (sum_of_dp < 0.00001 );
}

bool PID::Calibrate(double cte)
{

    bool is_a_cycle_finished = false;
          // while sum(dp)>0.00001: 
      //   for i in range(len(p)): 
      //       p[i] += dp[i] 
      //       robot = make_robot() 
      //       _, _, err = run(robot, p) 
      //       if err<best_err: 
      //           best_err = err 
      //           dp[i] *= 1.1 
      //       else: 
      //           p[i] -= 2*dp[i] 
      //           robot = make_robot() 
      //           _, _, err = run(robot, p) 
                 
      //           if err<best_err: 
      //               best_err = err 
      //               dp[i] *= 1.1 
      //           else: 
      //               p[i] += dp[i] 
      //               dp[i] *= 0.9



  switch(calibr_state)
  {

    case E_HILL_CLIMB_UP:
    {


        Kp += dp[0];
        Ki += dp[1];
        Kd += dp[2];

        calibr_state = E_HILL_CLIMB_UP_EVALUATION;
        break;

    }
    case E_HILL_CLIMB_UP_EVALUATION:
    {

        static int steps = 0;
        static double err = 0.0;

        steps++;

        if(steps > calibr_steps_n/2) //only calc error when it stabilizes a bit
        {
            err = (err + cte*cte)/2.0;

            if(steps > calibr_steps_n)
            {
                steps = 0;
                is_a_cycle_finished  = true;
                if(err < best_error)
                {
                    best_error = err;
                    for(auto &d :dp) { d *= 1.1;}

                    calibr_state = E_HILL_CLIMB_UP;
                }
                else
                {

                    Kp -= 2*dp[0];
                    Ki -= 2*dp[1];
                    Kd -= 2*dp[2];

                    calibr_state = E_HILL_DESCEND_EVALUATION;
                }
            }
        }

        break;

    }

    case E_HILL_DESCEND_EVALUATION:
    {

        static int steps = 0;
        static double err = 0.0;

        steps++;

        if(steps > calibr_steps_n/2) //only calc error when it stabilizes a bit
        {
            err = (err + cte*cte)/2.0;

            if(steps > calibr_steps_n)
            {
              //           if err<best_err: 
              //               best_err = err 
              //               dp[i] *= 1.1 
              //           else: 
              //               p[i] += dp[i] 
              //               dp[i] *= 0.9

                if(err < best_error)
                {
                    best_error = err;
                    for(auto &d :dp) { d *= 1.1;}
                }
                else
                {
                    //retore the original pid constants
                    Kp += dp[0];
                    Ki += dp[1];
                    Kd += dp[2];
                    //tighten search band
                    for(auto &d :dp) { d *= 0.9;}


                }

                is_a_cycle_finished  = true;
                steps = 0;
                calibr_state = E_HILL_CLIMB_UP;

            }
        }

        break;

    }

  }


        


    return is_a_cycle_finished;

}
