# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

In this project, a Model Predictive Control (MPC) agorithm is implemented for driving a simulated car around a race track. Actuations,
such as throttle and steering are used to for navigating the simulated world. The model is trying to follow waypoints on the map given 
to the vehicle at each communication session. These waypoints become the basis for planning the trajectory. 

Video capture of a demo drive is located here: https://youtu.be/XE3AcftklB4

The MPC model depends on the following state of the vehicle:
x - position of the vehicle along X axis.
y - position of the vehicle along Y axis.
psi - oritentation of the vehicle in coordinate system.
cte - Cross Track Error or distance to the reference line (ideal trajectory).
epsi - orientation error - angle between ideal orientation of the vehicle moving along
	the referenc line.
v - speed of the vehicle.

To control the vehicle, the model assume the following actuation parameters:
delta - steering angle of the vehicle's wheels.
a - throttle (accleration/deccelaration).

The range of both `a` and `delta` are in [-1, 1]. For the actual kinematics, the range of delta is in between [-25, 25 ] degrees.

Update equations of the state are as following ( I was following MPC_2_Line quiz):
```
      x1 = x0 + v0 * cos(psi0) * dt
      y1 = y0 + v0 * sin(psi0) * dt
      psi1 = psi0 + v0 * delta0 / Lf * dt
      v1 = v0 + a0 * dt
      cte1 = f0 - y0 + v0 * sin(epsi0) * dt
      epsi1 = psi0 - psides0 + v0 * delta0 / Lf * dt
```

Where index 0 correspond to time `t` and indix 1 correspond to the time at `t+1`. `Lf` is the parameter describing the vehicle, distance from
the center of the gravity to the steering axis of the vehicle. `dt` is the duration of time between `t` and `t+1`.

The cost of the trajectory depended on a number of parameters - cte, epsi, as well as difference between reference velocity and current velocity,
smoothness of acceleration and steering. All those factors were weighted.

The most important parameter, found in meta-parameter optimitzation, was the number of actuation steps `N` and duration of the between actuations.
I looked up some of the discussion on Udacity channel corresponding to the project and accepted `dt` as 100 ms for both optimality of duration as
well as convenience of implementation for another aspect of the project - latency of actuation. By keeping duration at the same value as the 
latency of actuation it was possible to postpone actuation by just one actuation step rather than trying to find a more complicated solution. 

The parameter `N` regulating the number of steps was more tricky to find. Despite suggested value of `10` I saw in the channel, I found the value 8
to be most optimal for high speed driving. I would like to strees, my goal in this project was to achieve the highest possible speed of the
vehicle on the race track. The value of `N` of 8 was maximizing the speed while still keeping the vehicle in the drivable area. Even slight changes (such
as `N=7` or `N=9` would change the balance significantly. I was able to reach 105 Miles per hour maximum speed while. I kept the reference veocity at
120 MPH, even though was was never able to achieve it. Still, having this velocity at more than achievable, as well as tuning other parameters resulted
in quite a high speed race. It was surpassing my PID-controlled car from the previous project by a factor or three.

I was able to set the `N` to up to 15, but only with smaller speeds of the vehicle. Something intuition suggested that the overall duration of planning
horizon should not suppass the reference path duration expressed in 6 waypoints given to the vehicle. In any case, with `N=8` I found a the best performance.

Other parameters I tuned manually after exhaustive search were the weight of the `CTE` error and `EPsi` error, as well as smoothness of change of 
steering and acceleration.

One of the largest impacts on the implementation of the project (and the aspect little discussed and only slightly suggested) was to change the
coordinate system to the one originating at the vehicle. This has conditioned the data well for the optimization to work. I transformed the coordinate 
of the waypoints to the new coordinate system and fitted polynomials to the transformed coordinates. MPC was also working with the coordinate system,
cendered at the vehicle. 

Without centralizing the coordinating system, it the conroller was very unstable and was only able to drive at slow speeds.

To deal with 100 ms latency I made two things:
1) Chose the `dt` interval aligned with the latency, thus `dt=0.1`.
2) Shifted the actuation inside the MPC optimization algorithm by one (see lines 114 at MPC.cpp).

I need to admit, that although the latency is well accounted with such a shift the visualization is not accounted for it as it display the trajectory that
was optimal 100 ms ago. I did not invest time into fixing at but it would be a nice proejct.

I also would like to state, that with this project I had the most fun. Good job, Udacity!


---

## Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
