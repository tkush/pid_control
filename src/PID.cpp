#include "PID.h"
#include <math.h>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(int maxIter, int mode ) {
	_p_error = 0;
	_d_error = 0;
	_i_error = 0;
	_returnVals = {0., 0.};

	_m1_max_speed = 60.; //mph
	_m1_speed_thres = 15.; //mph
	_m1_cte_thres = 0.25;

	// Mode - tuning or not 
	_mode = mode;

	// If tuning
	if ( _mode == 0 )
	{
		// initial guess for Kp, Ki, Kd, delta gains
		for (int i=0;i<3;i++)
		{
			if ( i == 0 )
			{
				_gains[i] = 0.2375; // a good initial guess
				_del_gains[i] = 1e-2;
			}
			else
			{
				_gains[i] = 0.;
				_del_gains[i] = 1e-4;
			}
			_ts[i] = un_init;
		}
		
		// Vehicle is out of lane if this threshold CTE is crossed
		_out_of_lane_cte = 4;

		// Accumulated error for tuning
		_error = 0.;

		// Previous best error
		_prev_error = 99999.;

		// Stop tuning when sum of _del_gains is less than this threshold
		_del_gains_thres = 1e-4;

		// Flag to update Ks cyclically per Twiddle algorithm
		_updateK = Kp;

		// Time step counter (each time this code is called)
		_count = 0;

		// Flag to indicate tuning in progress, restart simulator or done tuning
		_state = 0;

		// Num of time steps for accumulating error (while tuning)
		_num_steps_for_tuning = 10000;

		// Iteration counter
		// Each iteration is from start to restart while tuning
		_iter_count = 1;

		// Max iterations allowed
		_max_iter = maxIter;

		// Print out debug information
		_debug_print = true;
	}
	// Not tuning
	else
	{
		// These gains are arrived at by tuning the PID model using the 
		// Twiddle algorithm for 20 iterations starting with the initial
		// guess of Kp, Ki, Kd = 0.2375, 0., 0.
		_gains[0] = 0.2475;
		_gains[1] = 1e-4;
		_gains[2] = 8.9e-5;
	}
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	_gains[0] = Kp;
	_gains[1] = Ki;
	_gains[2] = Kd;	
}

void PID::UpdateError(double cte) {
	_d_error = _p_error - cte;
	_p_error = cte;
	_i_error += cte; 
}

double PID::TotalError() {
	return (-_gains[0]*_p_error - _gains[1]*_i_error - _gains[2]*_d_error);
}

void PID::Reset()
{
	_iter_count ++ ;
	_count = 0;
	if ( _iter_count > _max_iter )
		_state = 2;
	else
		_state = 1;
	_error = 0;
	_d_error = 0.;
	_i_error = 0.;
	_p_error = 0.;
}
std::vector<double> PID::Model2(double curSpeed, double curCTE){
	
	// Update current CTE
	UpdateError(curCTE);
	
	if ( _mode == 0 )
	{
		// Add to error: if the car is out of lane or stalled, add a large error
		if ( ( fabs( curCTE ) > _out_of_lane_cte || curSpeed < 0.05 ) &&
		       _iter_count != 1 )
		{
			cout << "Out of lane or stalled!" << endl;
			_error += 25 * ( _num_steps_for_tuning - _count ); // 5*5 for each remaining count
			_count = _num_steps_for_tuning - 1;
		}
		// Add to error: Add squared CTE
		else
			_error += curCTE * curCTE;

		// increment time step
		_count ++;

		// Start tuning if _num_steps_for_tuning time steps have passed by 
		// 				AND
		//				if _max_iter (max iterations) have not been reached
		if ( _iter_count <= _max_iter ) 
		{
			if ( _count == _num_steps_for_tuning )
			{
				double sum = _del_gains[0] + _del_gains[1] + _del_gains[2];
				if ( sum > _del_gains_thres )
				{
					// Normalize accumulated error 
					double normCTE = _error / _num_steps_for_tuning;

					// Print out some debug information
					if ( _debug_print )
					{
						cout << "Iteration# " << _iter_count << endl;
						cout << "-----------------------------------------------" << endl;
						cout << "CTE: " << normCTE << " BE: " << _prev_error << endl;
						cout << "\t Kp: " << _gains[0] << "\t d_Kp: " << _del_gains[0] << endl;
						cout << "\t Ki: " << _gains[1] << "\t d_Ki: " << _del_gains[1] << endl;
						cout << "\t Kd: " << _gains[2] << "\t d_Kd: " << _del_gains[2] << endl;
						cout << endl;
					}

					// Change update flags accordingly
					if ( _updateK == Kp )
						TuneK(0);
					else if ( _updateK == Ki )
						TuneK(1);
					else
						TuneK(2);
					
					// Reset parameters for next cycle of tuning
					Reset();
				}
			}
		}
		else
		{
			_state = 2; // done tuning
			cout << endl;
			cout << "***************************" << endl;
			cout << "Done tuning. Final values: " << endl;
			cout << "Kp: " << _gains[0] << endl;
			cout << "Ki: " << _gains[1] << endl;
			cout << "Kd: " << _gains[2] << endl;
			cout << "***************************" << endl;
			_mode = 1; 
		}
	}
	
	// Get steer angle
	_returnVals[0] = TotalError();
	if ( _returnVals[0] > 0.8 )
		_returnVals[0] = 0.8;
	else if ( _returnVals[0] < -0.8 )
		_returnVals[0] = -0.8;
	
	// Speed control: slow down when CTE is too high
	if ( fabs( curCTE ) > _m1_cte_thres )
	{
		if ( curSpeed > _m1_speed_thres )
			_returnVals[1] = -100; //full brake
		else 
			_returnVals[1] = 25; // quarter throttle
	}
	else
	{
		if ( curSpeed < _m1_max_speed )
			_returnVals[1] = 50; // half throttle
		else if ( curSpeed > _m1_max_speed )
			_returnVals[1] = 0.; // coast
	}

	return _returnVals;
}

void PID::TuneK(int i)
{
	double normCTE = _error / _num_steps_for_tuning;
	if ( _ts[i] == 0 )
	{
		_gains[i] += _del_gains[i];
		if ( _prev_error > normCTE)
			_prev_error = normCTE;
		_ts[i] = increase;		
	}
	else if ( _ts[i] == 1 )
	{
		if ( normCTE < _prev_error )
		{
			_del_gains[i] *= 1.1;
			_ts[i] = increase;
			_prev_error = normCTE;
			if (  _updateK == 0 )
			{
				_gains[1] += _del_gains[1];
				_ts[1] = increase;
				_updateK = Ki;
			}
			else if ( _updateK == 1 )
			{
				_gains[2] += _del_gains[2];
				_ts[2] = increase;
				_updateK = Kd;
			}
			else
			{
				_gains[0] += _del_gains[0];
				_updateK = Kp;
				_ts[0] = increase;
			}
		}
		else if ( normCTE > _prev_error )
		{
			_gains[i] -= 2*_del_gains[i];
			_ts[i] = decrease;
		}
	}
	else if ( _ts[i] == 2 )
	{
		if ( normCTE < _prev_error)
		{
			_del_gains[i] *= 1.05;
			_ts[i] = increase;
			_prev_error = normCTE;
			if (  _updateK == 0 )
			{
				_gains[1] += _del_gains[1];
				_updateK = Ki;
				_ts[1] = increase;
			}
			else if ( _updateK == 1 )
			{
				_gains[2] += _del_gains[2];
				_updateK = Kd;
				_ts[2] = increase;
			}
			else
			{
				_gains[0] += _del_gains[0];
				_updateK = Kp;
				_ts[2] = increase;
			}
		}
		else if ( normCTE > _prev_error)
		{
			_gains[i] += _del_gains[i];
			_del_gains[i] *= 0.9;
			_ts[i] = increase;
			if (  _updateK == 0 )
			{
				_gains[1] += _del_gains[1];
				_updateK = Ki;
				_ts[1] = increase;
			}
			else if ( _updateK == 1 )
			{
				_gains[2] += _del_gains[2];
				_updateK = Kd;
				_ts[2] = increase;
			}
			else
			{
				_gains[0] += _del_gains[0];
				_updateK = Kp;
				_ts[1] = increase;
			}
		}
	}
	return;
}

int PID::getState()
{
	return _state;
}

void PID::setState(int m)
{
	_state = m;
}

int PID::getMode()
{
	return _mode;
}



