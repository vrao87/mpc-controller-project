# CarNd MPC Controller project
  This project implements a model predictive controller to control a car to drive around the track in Udacity simulator. Simulator sends the current actuator information back to MPC using Web socket. In addition to it, following data is also sent from simulator:
  * `ptsx` (Array<float>) - The global x positions of the waypoints.
  * `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
     since y is the up-down direction.
  * `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format         expected in most mathemetical functions.
  * `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation]    (https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
  * `x` (float) - The global x position of the vehicle.
  * `y` (float) - The global y position of the vehicle.
  * `steering_angle` (float) - The current steering angle in **radians**.
  * `throttle` (float) - The current throttle value [-1, 1].
  * `speed` (float) - The current velocity in **mph**.
  
For this project I have used the code from MPC quiz presented in the classroom. Vehicle model and constraints are more or less reused from the quiz followed by parameter tuning.

## Code compilation
Code compiles without any error and no modification was done in CMakelists.txt.

## Implementation
The model used simple kinematic model presented in the class as well as used in the quiz. The state comprises of the following variables:
 * x, y - Position of car
 * psi  - Orientation angle of car
 * v    - car's velocity
 * cte  - cross track error
 * epsi - Orientation error
 
State update equations are the same as presented in class:
```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t-1] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
