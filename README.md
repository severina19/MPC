# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## The Model
### Vehicle Model
x_{t+1} = x_{t} + v_{t}*cos(\psi_{t})*dt
y_{t+1} = y_{t} + v_{t}*sin(\psi_{t})*dt
\psi_{t+1} = \psi_{t} − \frac{v_{t}}{L_{t}}\delta_{t}d_{t}
v_{t+1} = v_{t} + a_{t}*dt
cte_{t+1}= f(x_{t}) - y_{t} + (v_{t}*sin(e\psi_{t}))dt
e\psi_{t+1} = \psi_{t} - \psi des_{t} - \frac{v_{t}*\delta_{t}}{L_{f}} *dt

where f(x_{t}) = c_{0} + c_{1}*x + c_{2}*x^{2} + c_{3}*x^{3}
## Cost Function
As for the cost function, I created one based on the one described in the lesson, which includes following aspects: 
1. minimize difference between desired state and predicted state, which include the cross-track error, heading error, and the velocity error.
2. constrain erratic control inputs
3. make control decisions more consistent and smoother


## Latency 
To account for latency, the initial state was predicted for 100ms using the vehicle model, before providing it to the mpc solver. 