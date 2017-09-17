#ifndef PID_H
#define PID_H

#include <vector>
#include <iostream>

using namespace std; 

enum TwiddleStage
{
	un_init  = 0,
	increase = 1,
	decrease = 2
};

enum UpdateK
{
	Kp = 0,
	Ki = 1,
	Kd = 2
};

class PID {
public:
  /*
  * Constructor
  */
  PID(int maxIter, int mode);

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
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Model 2: Twiddle me controller!
  */
  std::vector<double> Model2(double curSpeed, double curCTE);

  /*
  * Model 2: Method to tune the gains
  */
  void TuneK(int i);

  /*
  * Model 2: Get the current state of the twiddle algo
  * 0 = tuning
  * 1 = restart
  * 2 = done tuning
  */
  int getState();

  /*
  * Model 2: Set the current state of the twiddle algo
  * 0 = tuning
  * 1 = restart
  * 2 = done tuning
  */
  void setState(int m);

  /*
  * Model 2: Get the current state of algorithm
  * 0 = tuning
  * 1 = tuning complete
  */
  int getMode();

  /*
  * Model 2: Reset counters, errors and state
  */
  void Reset();

private: 
	/*
	* Errors
	*/
	double _p_error;
	double _i_error;
	double _d_error;

	/*
	* Coefficients: gains[0] = Kp
	*			    gains[1] = Ki
	*			    gains[2] = Kd
	*/ 
	double _gains[3]; 
  double _best_gains[3];

	// Speed control parameters
  double _m1_max_speed;
	double _m1_speed_thres;
	double _m1_cte_thres;

	/*
	* Model 2: Parameters
	*
	*/
  int _max_iter;
  int _iter_count;
  double _out_of_lane_cte;
	int _count;
  int _mode; // 0: tuning, 1: run with hardcoded Kp, Ki, Kd
	double _error;
	int _num_steps_for_tuning;
	int _state; // 0: tuning, 1: restart, 2: done
	double _del_gains[3]; // delta gains for P, I and D
	double _prev_error;
	double _del_gains_thres;
  bool _debug_print;
	TwiddleStage _ts[3];
	UpdateK _updateK;

	// Return vector: [0]: steer value
	//                [1]: throttle value
	std::vector<double> _returnVals;
};

#endif /* PID_H */
