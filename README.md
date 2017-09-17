# PID control for autonomous control of a simulated vehicle
This code demonstrates the use of a simple PID controller to control the steering and throttle of a car driven in a simulator. The PID controller tries to minimize the "cross track error" which is the difference between the center of the car and the center of the track. 
Additionally, a simple PID tuning algorithm - called a "Twiddle" algorithm is implemneted here. 

## Cross track error
The "cross track error" or CTE is the signed distance between the center of the track and the center of the vehicle. This is a measure computed by the simulator and passed to the controller code at each step. 

![cte](/images/cte.png)

## PID controller
The PID controller is a standard Proportional, Integral and Derivate Controller defined as follows:

![controller](/images/controller.png)

The integral of the CTE is simply the sum of all previous and current CTEs. The derivative, likewise, is the difference between the current and last CTE. 

## Effect of P, I and D values on control
**Proportional co-efficient**
The P part of the controller serves to adjust the "responsiveness" of the controller to minimize the perceived error. That is, a higher value of P will make the controller calculate a steer angle that serves to minimize the CTE *faster*. However, increasing P too much will make the controller react faster but at the cost of overshooting the 0 CTE mark thereby making the system oscillate about the 0 CTE reference. 

TODO: insert images

**Integral co-efficient**
The I part of the controller serves to correct for error that add up over time. Since the proportional response can never really achieve a zero error response, the integral part is reponsible for keeping a tally of the errors that add up over time and correct for those. This is often called "bias correction" which basically means that the proportional control is indifferent to a bias in the error and the integral part tries to correct for that. 

**Derivative co-efficient"**
The D part of the controller is responsible for correcting when there is a rate of change of error for example disturbances in the system. A typical example of this is a discontinuous, sudden noise introduced in the system - the derivative response tries to correct for such an error. 

## Finding P, I, D values
For this project, the following approach is taken to find the optimized P, I and D co-efficients:
* The controller is run with non-zero P and zero I and D coefficients. 
* Multiple runs are made while manually tuning the P coefficient till the car can complete at least one full lap around the track
* With this P value as the starting point, the Twiddle algorithm is let loose on the controller. The Twiddle algorithm is decsribed in brief below
* Once a threshold criteria is reached (sum of changes in P, I, D components is less than a certain value or a maximum number of iterations are completed) the tuning is stopped

## Twiddle algorithm
The Twiddle algorithm is basically a local hill climber method. Pseudo code for this algorithm is described below: 
1. Run the algorithm with intial guess for P, I, D parameters
2. Perturb P, I or D parameters by a delta amount (cyclically)
3. Run the algorithm again. If error has decreased, increase the delta value by a small amount. This means that the increase in P/I/D parameters yields a decrease in error. We want to keep going in that direction. Goto 2. 
4. If the error has increased after (2), decrease the parameter (P/I/D) by twice the delta in (2). This means that the increase in P/I/D parameters yileds an increase in error. We want to go in the opposite direction. Goto 5. 
5. Run the algorithm again. If error has decreased, increase the delta value by a small amount. Goto 2. 
6. If the error has increased, reset the parameter (P/I/D) to the value in (1). Decrease the delta by a small amount. This means that we may have taken a very large step and the stepsize needs to be decreased. Goto 2. 

## Final parameters obtained after tuning
After an intial guess of PID = (0.2375, 0., 0.), the Twiddle algorithm is applied over 20 iterations for this simulator. Each iteration makes the car drive for about 1.5 laps. The final tuned values are **(0.2475, 1e-4, 8.9e-5)**.

Comments in the code are provided to aid in readability. 

