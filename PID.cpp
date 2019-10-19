#include "PID.h"
#include <vector>
#include <iostream>
#include <math.h>

using std::cout;
using std::endl;
using std::vector;

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  if (!is_initialized){
    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;
    this-> P = {Kp ,Ki, Kd};
    this-> p_error = 0;
    this-> i_error = 0;
    this-> d_error = 0;
    this-> dp = .1;
    this-> di = .1;
    this-> dd = .1;
    this-> DP = {dp,di,dd};
    is_initialized = 1;
	pid_optimized = false;
  }

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  this->d_error = cte-this->p_error;
  this->p_error = cte;
  this->i_error += cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double tot_err = -this->P[0]*p_error - this->P[1]*i_error - this->P[2]*d_error;
  if (tot_err >1)
    tot_err = 1;
  else if (tot_err <-1)
    tot_err = -1;
  return tot_err;  // TODO: Add your total error calc here!
}


double PID::Twiddle(double cte)
{
	double steer_value;
	double inc_dec = 0.1;
	if (fabs(cte) <= 0.1) // && cte >= -0.0001)
	{
	UpdateError(cte);
	steer_value = TotalError();
	cout << "					No PID change steering output : "<< steer_value<<"   CTE = "<< cte <<"   abs_cte = "<< fabs(cte) << endl;
    }
	else
	{
		double error = 999;
		  
		while (!pid_optimized)
		{
			for (int i = 0; i<(int)P.size(); ++i)
			{
			  this->P[i] += this->DP[i];
			  UpdateError(cte);
			  error = TotalError();
			  cout << "					1 > Increasing gain P["<< i << "] to :"<< P[i] << " gave steeting output : "<< error<<endl;
			  return error;
			  
			  if (abs(error) < abs(steer_value))
			  {
				  steer_value = error;
				  this->DP[i] *= (1 + inc_dec);			  
			  }
			  else
			  {
				  this->P[i] -= 2*this->DP[i];
				  UpdateError(cte);
				  error = TotalError();
				  cout << "					2 > Decreasing gain P["<< i << "] to :"<< P[i] << " gave steeting output : "<< error<<endl;
				  if (abs(error) < abs(steer_value))
				  {
					  steer_value = error;
					  this->DP[i] *= (1 + inc_dec);
				  }
				  else
				  {
					  this->P[i] += DP[i];
					  this->DP[i] *= (1 - inc_dec);
				  }
			  }
			}
			if ( (this->DP[0] + this->DP[1] + this->DP[2])<0.01 )
			{
				pid_optimized = true;
				cout<<"PID optimized" << endl ;
			}
			else
			{
				pid_optimized = false;
				cout<< "PID not optimized" <<endl;
			}		
		}
		
	}
return steer_value;	
}
