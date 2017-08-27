#include "PID.h"
#include <cmath>
#include <iostream>
/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
	// initialize PID class parameters
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	p_error = i_error = d_error = 0;
	best_error = numeric_limits<double>::max();
	total_error = 0;
	
	// init PID twiddle parameters
	dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};
	step = 1;
	param_idx = 2;
	
	PID_add = false;
	settle_steps = 100;
	eval_steps = 2000;  //this time should cover roughly 1 road loop
}

void PID::UpdateError(double cte) {
	if (step == 1) {
		d_error = 0.0;
	}
	else {
		d_error = cte - p_error;
	}
	p_error = cte;
	i_error += cte;
	
	/* LOGICS
	To evaluate per Twiddle tuning, we need a period to evaluate the error
	At the end of each period
		Steps:  0---settle_steps------------------------eval_steps           ->0---
		Action: 0---[start updating total error]--------[tune PID and loop back]---
	*/
	// update total error
	if (step % (settle_steps + eval_steps) > settle_steps){
        total_error += pow(cte,2);
    }
	
	// if end of eval_steps, perform Twiddle
	if (step % (settle_steps + eval_steps) == 0){
		cout << "step: " << step << endl;
        cout << "total error: " << total_error << endl;
        cout << "best error: " << best_error << endl;
        
        
        if (total_error < best_error){
        	//if error is reduced, increase dp and move to the next parameter in this same loop
        	//also reset the PID_add state memory to true.
        	best_error = total_error;
        	if (step > (settle_steps + eval_steps)){
        		dp[param_idx] *= 1.1;
        	}
        	param_idx = (param_idx + 1) % 3;
        	AdjPID(param_idx, dp[param_idx]);
            PID_add = true;
        }
        else{  //if error is not reduced, check whether we have added or subtracted dp
        	if (PID_add){
        		//if added, move to the other direction
        		AdjPID(param_idx, - 2 * dp[param_idx]);
            	PID_add = false;
        	}
        	else {
        		//if subtracted, balance it back and reduce step size.
        		//Then move to the next parameter
        		AdjPID(param_idx, dp[param_idx]);
        		dp[param_idx] *= 0.9;
        		
        		param_idx = (param_idx + 1) % 3;
        		AdjPID(param_idx, dp[param_idx]);
        		PID_add = true;
        	}
        }
        
        total_error = 0;
        cout << "new parameters after adjusting PID index " << param_idx << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;
        cout << "dp: [" << dp[0] << ", " <<  dp[1] << ", " << dp[2] << "]" << endl;
	}
	step += 1;
}



double PID::TotalError() {
	return - Kp * p_error - Kd * d_error - Ki * i_error;;
}

void PID::AdjPID(int index, double x) {
	switch(index){
		case 0:
			Kp += x;
			break;
		case 1:
			Ki += x;
			break;
		case 2:
			Kd += x;
			break;
		default:
			std::cout << "AddToParameterAtIndex: index out of bounds " << index << endl;
	}
}