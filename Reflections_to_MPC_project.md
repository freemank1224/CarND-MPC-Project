# Reflections to MPC project

Udacity SDC

---

## The model

 - The state variables of the vehicle was $ [x, y, \psi, v, cte, e_{\psi}] $.
 - The control inputs or actuators: $ [\delta, a] $
 - The update equations:
 $x_{t+1} = x+t + v_tcos(\psi)dt$
 $y_{t+1} = y_t + v_tsin(\psi)dt$
 $\psi_{t+1} = \psi_t + \frac{v_t}{L_f}\delta_tdt$
 $cte_{t+1} = f(x_t) - y_t + v_tsin(e_{\psi_{t}})dt$
 $c_{\psi} = \psi_t - \psi_{des_t} + \frac{v_t}{L_f}\delta_t dt$

## Timestep Length and Elapsed Duration ($N$ & $dt$)
I have tested several $N$ and $dt$ combinations in the test running. 
According to my tuning, larger &N& values were not alwasys good for vehicle handling. Larger $N$ values means more computation that was not harm for real-time performance of the algorithm.
On the other hand, $dt$ indicateed the time inteval between to consecutive control signals. It should be small enough to ensure the control signals can be applied to the vehicle and response timely. $dt$ should decrese with the vehicle speed.
After detailed comparison of different $N$ and $dt$ pairs, I finaly chose $dt = 0.05 sec$ and $N = 12$.

## Polynomial Fitting and MPC Preprocessing
The polynomial fitting was performed based on the "ptsx" and "ptsy" value which were sent back from the simulator. We have used 3rd order polynomial fitting to find an optimal route. Before fitting the curve, I have tranformed these points from global cordinate to the vehicle cordinate (called local cordinates in my code file).
Cordination transformation has been performed for display the reference lines in the simulator as well as facilitate vehicle state description. After transformation, the state variables became $[0, 0, 0, v, cte, e_{\psi}]$ since in the local reference frame, the vehicle was always located at original point and heading to the default orientation. 


## Model Predictive Control with Latency
In my solution, I implemented an latency processing function in MPC.cpp as a method of the MPC class. I used vehicle's kinematic model to compute the state transition during the latency time, 100ms. This part of code was listed below.

```C++
  px = px + v*cos(psi)*latencyTime; 
  py = py + v*sin(psi)*latencyTime;
  cte= cte + v*sin(epsi)*latencyTime;
  epsi = epsi + v*delta*latencyTime/Lf;
  psi = psi + v*delta*latencyTime/Lf;
  v = v + acc*latencyTime;
```
In above code, the "latencyTime" was equal to 100ms.
All of the operations were performed under the vhicle coordinate. Then the updated state variables was sent to the optimizer as current state, which compensated the system latency.

