# Model Predictive Control (MPC) Project

## Rubric points

Here I will address the rubric points for this project.

### Your code should compile

> Code must compile without errors with cmake and make.

The code should be compilable with `cmake` and `make`. The only modifications were made for the `CMakeLists.txt` are:

1. The link directory for `libuv` was updated to reflect version 1.12.0 of the library.
2. The `sources` variable includes `src/Navigator.cpp` file as well.

### The Model

> Student describes their model in detail. This includes the state, actuators and update equations.

The implemented model is a simple kinematic model of a car. The state includes position of the car *x* and *y*, orientation *psi*, and velocity *v*. It also includes the cross-track error *cte*, and orientation error *psie*. The actuators include the steering angle *delta* and accelleration *a*.

The update equations for the time *t+1* for the car model are the following:

- x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
- y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
- psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
- v[t+1] = v[t] + a[t] * dt
- cte[t+1] = fwaypoints(x[t]) - y[t] + v[t] * sin(psie[t]) * dt
- psie[t+1] = psi[t] - psides[t] + v[t] / Lf * delta[t] * dt

Where *dt* is elapsed time between state *t* and *t+1*, and *Lf* is the distance between center of mass of the car and its front.

The *fwaypoints(x)* is a function that returns *y* coordinate of the desired trajectory for the input *x* coordinate.

The *psides* is a desired orientation at position *x*.

The cost function for the model is a weighted sum of squares for the following items:

- cross track error *cte*
- orientation error *psie*
- velocity error, which is a difference between reference velocity (constant), and velocity at time t *v*
- steering actuator value *steer*
- acceleration actuator value *acc*
- delta between previous steering value and current value *steer_gap*
- delta between previous acceleration value and current value *acc_gap*

The weights were adjusted in such a way that solver would prefer following in decreasing priority order:

1. Minimize the delta between values of sequential activations of the actuators, so that car does not change steering angle abruptly, nor does accelerate/deccelerate chaotically
2. Minimize the delta between acceleration activations
2. Minimize the steering values
3. Minimize the cross-track error and orientation error
4. Minimize the acceleration value
5. Minimize the velocity error

Initially the cross track and orientation errors had much bigger weight than deltas between sequential activations, and it worked well when there were no latency, but with incorporated latency, it just did not work - car was swinging across the track with huge steering angles, and falling off almost immediately. So the priorities were changed, and the values were adjusted.

The idea behind is also *it is better to drive slower but safer* and to act smoothly.


### Timestep Length and Elapsed Duration (N & dt)

> Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The MPC is working by applying the car model over time to calculate future (predicted) trajectory given the input state and activations of actuators applied over time. First state is usually the current car position, orientation and velocity. Then the actuators are applied and next state is calculated, and so on. The values of actuators are then optimized to minimize the cost function, which is usually defined as a difference between desired trajectory and predicted trajectory.

The predicted trajectory is calculated for *N* steps into future with *dt* seconds between each step. The higher *N* the further in future trajectory can be predicted, more expensive computation is. The lower *dt*, the more precise trajectory can be predicted (when there is no latency). Based on experiments, a good value for the time horizon, which is a product of *dt* and *N*, is about 1 second. Based on that, the low limit for the *dt* is latency, which is *0.1*, which brings *N=10*.

The time horizon could not be too near (e.g. *N=5* and *dt=0.1* does not work at all - car almost immediately steers away from the track). Having it too far does not work pretty well as well. For the latter what happens is that one of the input parameters is the list of the waypoints. Those waypoints are not consumed as is by the MPC solver, but rather a 3-rd degree polynomial is fitted to its coordinates and that serves as an input for the solver. There are a few problems here. One is that there are not many waypoints, and we should not predict into the future further than those waypoints are located, as otherwise we would not be able to calculate error properly (because any value outside of the waypoints range can be valid). Another problem is that when we fit a polynomial, it is an approximation, and actual trajectory of the waypoints might be more complex than 3-rd degree polynomial, so the MPC might try to optimize the predicted trajectory to minimize the big error at the end of the trajectory, while sabotaging the error at the beginning of it, which may lead the car to steer off the track. I think this is what was happening for the combination of *N=10* and *dt=0.1*. Using *N=5* and *dt=0.2* did not work well as well, probably because the predicted trajectory become too coarse.


### Polynomial Fitting and MPC Preprocessing

> A polynomial is fitted to waypoints.

A 3-rd degree polynomial is fitted to waypoints, which seems a good approximation for the track curvatives.

> If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The coordinates of the waypoints were converted from the global map coordinates to the local car coordinates, so we benefit from the following:

- simpler calculations for the cross track error
- simpler calculations for the orientation error
- latency model can be incorporate easily
- to display waypoints and predicted trajectory, we need to provide coordinates in car's local system

The following values were preprocessed as well:

- *steering_angle* value received from the simulator is negated as positive value corresponds to turning clock wise, while positive *psi* corresponds to turning counter clock wise;
- *steering_angle* value for the actuator is negated as well (same reason as above), and the value is scaled to be in the range *[-1; 1]*, as the model outputs the steering angle in radians in the range [-25 deg; 25 deg];
- car position *x*, *y*, and orientation *psi* is preprocessed to incorporate the latency model.

To incorporate the latency model, the same update equations are applied to the incoming values of the car position and orientation, the *dt* is equal to *latency* value (which is known). The speed of the car for simplicity is assumed to not change, because we don't have the acceleration value (only throttle, which is not the same). The reasoning is following: the latency is in the actuators, so when we send activation values, car has already moved *latency* seconds, so what we need is to start prediction now, but as if car would be in the position that it would have *latency* seconds from now. Another way how it can be probably simulated is by adding latency value to the *dt* on the first iteration when calculating the feedback function (but keeping iterations *[2..N]* with *dt*), and taking the second activation as result (instead of the first one).

### Model Predictive Control with Latency

> The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

The latency issue was described in the previous section about preprocessing.

### The vehicle must successfully drive a lap around the track.

> No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).
> The car can't go over the curb, but, driving on the lines before the curb is ok.

[It does!](https://youtu.be/PPJY2pJM9-g)