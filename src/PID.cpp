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

    PID::k_vec = {Kp, Ki, Kd};
    PID::dp = {1,1,1};
    PID::calibr_steps_n = 1000;
    PID::calibr_state = E_HILL_CLIMB_UP;
    PID::best_error = 0xffffffff;
}

void PID::UpdateError(double cte) {
    static double past_cte = 0;
    static double sum_cte = 0;

    sum_cte += cte;
    p_error = k_vec[0]*cte;
    i_error = k_vec[1]*sum_cte ;
    d_error = k_vec[2]*(cte-past_cte);

    past_cte = cte;

    std::cout <<  "p error: "<<p_error<<"; i error: "<<i_error<<"; d error: "<< d_error << std::endl;
}

double PID::TotalError() {
    return p_error + i_error + d_error;
}             

bool PID::isCalibrDone (void)
{
    double sum_of_dp = std::accumulate(dp.begin(), dp.end(), 0.0);
    std::cout << "@isCalibrDone dp: "<<dp[0]<<", "<<dp[1]<<", "<<dp[2]<<std::endl;
    std::cout << "@isCalibrDone sum of dp: "<<(sum_of_dp)<<std::endl;
    return (sum_of_dp < 0.00001 );
}

#define PID_STABALIZATION_TIME 20
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



  std::cout << "calibr_state: "<<calibr_state<<std::endl;
  switch(calibr_state)
  {
    static int idx = -1;

    case E_HILL_CLIMB_UP:
    {

        idx = (idx + 1)%dp.size();
        k_vec[idx] += dp[idx];

        calibr_state = E_HILL_CLIMB_UP_EVALUATION;
        break;

    }
    case E_HILL_CLIMB_UP_EVALUATION:
    {

        std::cout << "index: "<<idx<<std::endl;
        static int steps = 0;
        // static double err = 0.0;
        static double err_sum = 0.0;

        steps++;

        std::cout << "HILL_CLIMB_UP steps: "<<steps<<std::endl;

        if(steps > PID_STABALIZATION_TIME) //only calc error when it stabilizes a bit
        {
            err_sum +=  cte*cte;
            std::cout<<"current err: " << err_sum/(steps-PID_STABALIZATION_TIME)<<std::endl;
            std::cout<<"best err: " << best_error<<std::endl;

            if(steps > calibr_steps_n)
            {

                double err_avg = err_sum/(steps-PID_STABALIZATION_TIME);
                if(err_avg < best_error)
                {
                    best_error = err_avg;
                    dp[idx] *= 1.1;
                    calibr_state = E_HILL_CLIMB_UP;
                }
                else
                {
                    k_vec[idx] -= 2*dp[idx];

                    calibr_state = E_HILL_DESCEND_EVALUATION;
                }

                steps = 0;
                is_a_cycle_finished  = true;
                err_sum = 0;
            }
        }

        break;

    }

    case E_HILL_DESCEND_EVALUATION:
    {

        static int steps = 0;
        // static double err = 0.0;
        static double err_sum = 0.0;

        steps++;
        std::cout << "index: "<<idx<<std::endl;
        std::cout << "HILL_DESCEND steps: "<<steps<<std::endl;

        if(steps > PID_STABALIZATION_TIME) //only calc error when it stabilizes a bit
        {
            err_sum +=  cte*cte;

            std::cout<<"current err: " << err_sum/(steps-PID_STABALIZATION_TIME)<<std::endl;
            std::cout<<"best err: " << best_error<<std::endl;

            if(steps > calibr_steps_n)
            {
              //           if err<best_err: 
              //               best_err = err 
              //               dp[i] *= 1.1 
              //           else: 
              //               p[i] += dp[i] 
              //               dp[i] *= 0.9
                double err_avg = err_sum/(steps-PID_STABALIZATION_TIME);

                if(err_avg < best_error)
                {
                    best_error = err_avg;
                    dp[idx] *= 1.1;
                }
                else
                {
                    //retore the original pid constants
                    k_vec[idx] += dp[idx];

                    //tighten search band
                    dp[idx] *= 0.9;


                }

                is_a_cycle_finished  = true;
                steps = 0;
                err_sum = 0;
                calibr_state = E_HILL_CLIMB_UP;

            }
        }

        break;

    }

  }


        


    return is_a_cycle_finished;

}
