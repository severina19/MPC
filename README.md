# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## The Model
### Vehicle Model
I used the vehicle model introduced in the lesson:

x[t+1] = x[t] + v[t]*cos(psi[t])*dt

y[t+1] = y[t] + v[t]*sin(psi[t])*dt

psi[t+1] = psi[t] − v[t]/ Lf*delta[t]*dt

v[t+1] = v[t] + a[t]*dt

cte[t+1]= f(x[t]) - y[t] + (v[t]*sin(epsi[t]))*dt

epsi[t+1] = psi[t] - psides[t] - v[t]*delta[t]/Lf *dt

The state of the model include: 
* x: x position of the car
* y: y position of the car
* psi: heading direction of car
* v: velocity of car
* cte: cross track error 
* epsi: orientation error

And the actuators include:
* a: acceleration of the car
* delta: steering angle of the car

The variable Lf is the distance between the mass of the car and the front wheels, and f(x[t]) is calculated as f(x[t]) = c0 + c1*x + c2*x^2 + c3*x^3, where c0, c1, c2 and c3 are the coefficients of the desired trajectory. 

## Cost Function
The mpc solver tries to minimized a defined cost function and thus to find out the optimal value of the control input, including the acceleration and the steering angle. As for the cost function, I created one based on the one described in the lesson, which includes following aspects: 

1. minimize difference between desired state and predicted state, which include the cross-track error, heading error, and the velocity error.
2. constrain erratic control inputs
3. make control decisions more consistent and smoother


## Timestep Length and Elapsed Duration (N & dt)
I used a time duration of 10s and a timestep length of 0.1s. The values are determined through trial and error. A larger duration N would result in higher computational time and is also not necessary, as the environment will be changing anyways even if we have a larger prediction horizon. A small timestep length dt would result in more accurate prediction but cause higher computational time, and a controller which lags behind will also result in undesired behavior. A larg timestep length would result in inaccurate prediction.


## Polynomial Fitting and MPC Preprocessing
The simulator provides waypoints in x and y direction in global coordinate system. First, the waypoints are transformed into vehicle coordinate system, and then a 3rd degree polynomial is used to fit the desired trajectory. The coefficients are provided to the mpc solver to calculate cte and epsi.

## Latency 
To account for latency, the initial state was predicted for 100ms using the vehicle model, before providing it to the mpc solver. 