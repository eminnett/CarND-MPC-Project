#### Student describes their model in detail. This includes the state, actuators and update equations.
The model used in my MPDC implementation is a global kinematic model (lines 126-131 of MPC.cpp) including steering and throttle actuators.

```
x_1 = x_0 + v_0 * CppAD::cos(psi_0) * dt;
y_1 = y_0 + v_0 * CppAD::sin(psi_0) * dt;
```

These first two equations of the model predict the new position of the vehicle based on the current position, current velocity, current orientation of the vehicle, and the elapsed time dt.


```
psi_1 = psi_0 - v_0 * delta_0 / Lf * dt;
```

This third equation of the model predicts the new steering angle given vehicle's current orientation, current velocity of the vehicle, and the elapsed time dt. `Lf` is the distance between the front of the vehicle and the vehicle's centre of gravity. This value impacts the turning rate of the vehicle and the new orientation of the vehicle given the other factors in the equation. The actuator `delta_0` represents the current steering angle of the vehicle which directly impacts the new vehicle orientation.

```
v_1 = v_0 + a_0 * dt;
```

This fourth equation models the change in velocity as time passes. It is a simple application of kinematic motion where the change is velocity is acceleration multiplied by the change in time. The actuator `a_0` is the current acceleration of the vehicle and represents the throttle actuation both for accelerating (when `a_0` is positive) and braking (when `a_0` is negative).

```
cte_1  = (f_0 - y_0) + (v_0 * CppAD::sin(epsi_0) * dt);
epsi_1 = (psi_0 - psi_desired_0) - v_0 * delta_0 / Lf * dt;
```

The remaining two equations calculate the new cross track error and orientation error of the vehicle. Given the difference between the desired y position, `f_0` (as calculated from the polynomial fit), and the current y position `y_0` as well as the change in the y position (given the current velocity, current orientation error `epsi_0`, and elapsed time dt) we can calculate what the new cross track error will be. The polynomial fit also allows us to calculate the desired orientation `psi_desired_0`. The difference between the current orientation `psi_0` and desired orientation along with the change in orientation given the current velocity, current steering angle, `Lf`, and elapsed time dt we can determine what the new orientation error will be. These last two equations are particularly important as they provide a feedback loop that helps determine what the best steering and throttle actuations should be to minimise these errors.

#### Student discusses the reasoning behind the chosen N (timestep length) and dt (timestep frequency) values.

When choosing the values of the timestep length `N` and timestep frequency `dt` I decided to start with the values of `N` and `dt` from the MPC quiz solution. I figured this was as a good a place to start as any other. These values of 25 for `N` and 0.05 for `dt` lead to the car oscilating left and right. As soon as the car approached the first corner, it promptly drove off the edge.

To find appropriate values, I repeatedly decreased `N` by 5 and then increased `dt` by 0.05 until the vehicle drove safely. The car continued to oscilate left and right though the oscilations dampened as `N` decreased.

I wanted to explore having a prediction horizon greater than 1 to see what would happen, but found the best performance was to be had when `N` was 10 and `dt` was 0.1. This results in a prediction horizon of exactly 1.

My notes for each iteration can be found below:

- N: 25, dt: 0.05: The car oscillates left and right
- N: 20, dt: 0.05: The car oscillates left and right
- N: 15, dt: 0.05: The car oscillates left and right but a little less
- N: 15, dt: 0.1: The car oscillates left and right but a little less
- N: 10, dt: 0.1: The car drives safely. The best performance appears to be for N: 10, dt: 0.1. I wanted to explore having a prediction horizon T value greater than 1, but found when it is too large, the model performed poorly.


#### Student provides details on how they deal with latency.

To begin with, I felt it was too arbitrary to use a reference velocity (MPC.cpp line 33) less than the maximum velocity supported by the simulator. I quickly found that allowing the vehicle to drive so fast resulted quite poor performance. The vehicle was unable to complete a lap of the track at this speed. Rather than decreasing the reference velocity, I chose instead to deal with the issues caused by the communication latency by introducing cost multipliers for each term of the cost function. I added different terms for each element of the function and spent several hours exploring different values for these multipliers. I found the following multipliers resulted in the best compromise between safety, smoothness, and speed:

```
const int cte_cost_multiplier       = 5000;
const int epsi_cost_multiplier      = 1000;
const int steering_cost_multiplier  = 50000;
const int throttle_cost_multiplier  = 75;
const int delta_cost_multiplier     = 100;
const int a_cost_multiplier         = 10;`
```

*Please see these values in context in MPC.cpp lines 61-83 for more detail.*

By heavily penalising changes to the steering angle and cross track error, I was able to get the vehicle to drive smoothly and avoid the left and right oscilations introduced by the communication latency. The remaining cost multipliers helped slow down the vehicle allowing it to corner and complete a lap of the track safely and consistently.


#### References:
Elements of my implementation have been influenced by the MPC quiz solution: https://github.com/eminnett/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp
