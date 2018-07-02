# Model Predictive Controller project

This repository contains implementation of the Model Predictive Controller that allows to control the car driven on the race track in simulation environment. The race track is circular with several left and right corners with different curvature radiuses.

## Model Description

The state  of the vehicle is defined as {x, y, θ, v), where (x, y, θ) represent the position and orientation of vehicle in the global (map) reference frame, and v represent current velocity. For controlling the vehicle on the track a kinematic model is used, that allows continiously evaluate how the state of the vehicle evolves over time from the current position, and how to adjust the vehicle control actuators (steering angle **delta** and acceleration **a**) in order to adapt the state to continiously chaning road environment. The adjustment of controls is based on evaluation of the lateral offset of vehicle from the lane center line (Cross Track Error (cte)), and the orientation error (epsi). 

Updating the state for each next timestep (t+1) from current timestep (t) is done via the following equations:

    x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v[t+1] = v[t] + a[t] * dt
    cte[t+1] = cte[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] / Lf * delta[t] * dt

where 
 - **Lf** is the distance from the front of the vehicle to its center of gravity, used to take into account the size of vehilce, that affects its turning radius.
- **psides** is desired orientation

## Reference trajectory points preprocessing

The reference trajectory for drivng the car on a track is provided by simulator as a sequence of waypoints ({x[0], y[0]}, ..., {x[k], y[k]}, ..., {x[n], y[n]}) in the global (map) reference frame, as two-dimensional Cartesian coordinate system. Thus before using this trajectory for computations, it is needed to transform all its points into vehile coordinate system using following equations:

   point_car_x = (point_map_x - car_position_x) * cos(psi) + (point_map_y - car_position_y) * sin(psi);
   point_car_y = (point_map_y - car_position_y) * cos(psi) - (point_map_x - car_position_x) * sin(psi);


## How MPC Controller tunning parameters were selected

### Prediction Horizon

The Prediction Horizom is the duration over which controller predicts future states. It implies the number of timesteps N and duration of each timestep dt. This means that bigger values of **N** and/or smaller values of **dt** will result in higher computational costs.

Selection of optimal value for N and dt parameters was done empirically. Several values between 15 and 25 have beed tried for **N** parameter, and between 0.03 and 0.1 for **dt** parameter. The best result was achieved with N=20 and dt=0.05, resulting in a prediction horizom of 1 second. Additionally value 0.05 for dt parameter allowed to syncronize the timesteps with defined  control actuatoations latency (0.1sec)

Values for **dt** bigger than 0.05 did not preform well, because controller was not able to react in time on dynamically chaning curvatures of the track (was highly noticable in sharp curves). On other hand values smaller than 0.05 caused too high computational load on the system. 

Values for parameter **N** smaller than 20 resulted in a jerky behaviour of the car on track, and values higher than 20 were causing inability of controller to correctly predict the optimal trajectory in the sharp curves, thus car was constantly leaving the track in such places.

### Latency compensation

Additional requirement to implemented model was to handle the control actuations latency (with predefined value 100ms), as an emulation of delay between triggering the control actuators and start of actual effect of on the system. 
In such enviroments the control model which does not take the actuations latency into account, is not able to properly control the car (sudden oscillations and/or incorect predicted rajectory occur). In order to 
properly handle emulated actuations latency I used following approach: the position of the car is  estimated from its current position (based on its speed and steering angle (delta)) forward until the timestep  when actuations are expected to have an effect (100ms). And then optimal trajectory is computed starting from that position.

### Additional parameters

Additionaly, to make the behaviour of the car on the road smoother and comfortable for potential passengers I have introduced **STEERING_TRANSITION_SMOOTHNESS** and **ACCELERATION_SMOOTHNESS** parameters with empirically selected values 600 and 15 accordingly. These are applied dutring the calculation of cost function.

---

## Setup instructions

This project requires the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Scripts install-ubuntu.sh and install-mac.sh can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems accordingly. For windows use either Docker, VMware, or [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/)

## Application build dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* Ipopt and CppAD
  * Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.



## Build Instructions

Once installation of all dependencies is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make

Alternatively build script has been included to simplify this process, it can be leveraged by executing the following in the top directory of the project:

   ./build.sh
   
## Execution instructions

1. Start created MPC Controller application from build folder:
  
   ./mpc
   
2. Start downloaded Term 2 Simulator application. Select requried resolution and press Play button.
   Switch to 'Project 5: MPC Controller' using right arrow an press Select button.
   To run the simulation press Start button. 
