# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[](https://media.giphy.com/media/KEZ5WXWiCP9Umb6kMj/giphy.gif)

In this project, a behavior and path planning agent is programmed to drive in a simulated environment. The agent represents a car on a 3-lane race track where other agent-cars also drive.

### Goals
The goal of the project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The agent car is provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Demo
A video of the agent driving on a track according to the above conditions is available [here](https://youtu.be/O1Rsy83SXwg).

### Original project details
The original repo of the project without implementation of the agent behavior is available [here](https://github.com/udacity/CarND-Path-Planning-Project).
   
#### Simulator.
In order to reproduce the project, one must download the Term3 Simulator which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Project Rubric and Details of Implementation

### Project Rubric

1. The code compiles correctly. Fulfilled. The CMakeLists.txt is almost unchaged from the original project with the exception of code-optimization `-O0` directive for the purpose of debugging. Also, for the purpose of debugging one can set a CMake-variable `-DDEBUG_MODE` in order for the code to print some debugging information to console.

2. The car is able to drive at least 4.32 miles without incident. Fulfilled. The car correctly keeps safe distance with the car ahead of it and chooses optimal right and left turn based on traffic ahead provided to its sensor fusion. This happens in the majority of cases. Throughout the testing, however, I have seen some failure cases. These were mostly related to other cars making sudden left/right lane changes and the ego-car not being able to react within 1 second (time intertia for the maneuver). One drawback of the current scheme of path planning is the continuity of trajectories between updates which introduces such inertia. For exmaple if 50 trajectory points were generated, an only 3 were used between updates, the rest of 47 trajectory points stay the same even in the case the behavior needs to change. All other cases are handled correctly.

3. The car drives according to the speed limit. Fulfilled. The car always tried to go as fast as possible but never exceeds the speed limit. Its reference velocity is set at 48MPH, which, with various spurious lateral and longitudinal accelerations, does not make it go above 50MPH limit.

4. Max acceleration and jerk are not exceeded. Fulfilled. I was using suggested starting code from project Q&A [session](https://www.youtube.com/watch?time_continue=1&v=7sI3VHFPP0w) which already took great care in implementing smooth trajectories with the help of Spline library. The car exceeded neither acceleration (10 m/s^2) nor jerk (10 m/s^3) thresholds; the car at the same time the car had a good starting acceleration to get to its nominal speed just below the speed limit.

5. Car does not have collisions. Fulfilled for the distance required. In the video above, the car makes safe left/right lane changes, and keeps safe distance from the car ahead of it. As was described in (1), some incidents can sometimes be observed, but those are unlikely and usually require much longer observation time.

6. The car stays in its lane, except for the time between changing lanes. Fulfilled. Similarly to (4), I was using suggested starting code from project Q&A session that was already taking care of lane changes maneveurs. The lane change takes less than 2 seconds to execute.

7. The car is able to change lanes. Fulfilled. The car makes safe left/right lane changes depending on traffic situation. The selection of the lane-change direction depends on the degree of clearance of left/right lane from other traffic.

### Reflection on Implementation

The code of the project reflects many course objectives. Some part of the code, such as smooth trajectory generation, were borrowed from discussion of the Q&A [session](https://www.youtube.com/watch?time_continue=1&v=7sI3VHFPP0w). The following list describes the code according to course objectives.

1. Prediction of future positions of other cars based on sensor fusion is coded on lines [110-160](https://github.com/sfefilatyev/CarND-Path-Planning-Project/blob/master/src/main.cpp#L110). The car uses `sensor fusion` information in order to predict the traffic one second ahead of time. This prediction takes specific focus on the car ahead of the ego-car (by setting `too_close` variable), as well as populating two other variables `closest_car_right_lane` and `closest_car_left_lane` variables that describe the cleared distance a car can safely travel from the ego s-Frenet-point on the right and left lane to it correspondingly. Similarly to Q&A session, the safe distance to the car ahead of us is defined at 30 meters. The safe distance to make a left or right lane change is defined as 30 meters in ahead of the ego car and 15 meters in behind the ego car. These distances are in relation to the s-axis of Frenet coordinate system. I think, the requirement of 15 meters in the cleared distance behind the ego vehicle can be further reduced b/c based on the behavior of the other cars on the race track I've observed they usually go significantely slower than the speed limit.

2. Poor's Man Behavior Planning. The car has three modes of driving - keep in its lane, make a left lane change, make a right lane change. The code for behavior planning is located in lines [164-194](https://github.com/sfefilatyev/CarND-Path-Planning-Project/blob/master/src/main.cpp#L168). Althought I feel the original Behavior Planning quiz [code](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/56274ea4-277d-4d1e-bd95-ce5afbad64fd/concepts/2c4a8fdd-4072-425f-b5ae-95849d5fd4d8) was very appropriate to be used in this project (especially the code from the Vehicle class), time constaints pushed me to make a simiplified version. In my project's case, I had a 'poor's man' implementation of cost functions. The cost function were just implemented as `closest_car_right_lane` and `closest_car_left_lane` variables which were needed in order to select a right/left lane change based on the traffic in those lanes. The direction for a lane change, thus, was selected based on the maximum distance without cars in those lanes. The change was only triggered if the ego-car was following a vehicle in its lane (with `too_close` set to true). Despite such simplifications, the car is quite able to move fast and safe for the required distance.

From the drawbacks of such implementation I want to note that if the car is situated in the left lane and is "cornered" by two other car from front and  right, it will not make any changes, and will wait until either ego lane or left lane clears. Thus, it cannot deacelerate in order to pass the cars on the right. Same situation happens if the car is cornered in its right-most lane.

3. Path generation. The code for it is located in lines [196-317](https://github.com/sfefilatyev/CarND-Path-Planning-Project/blob/master/src/main.cpp#L196). I was using code from Q&A session that took care of smooth path generation that does not exceed acceleration and jerk threshols. The original code from Q&A was integrated to work with the code from prediction and behavior planning submodules in order to meet requirements.

## Conclusion
This is a project is a combination of multiple self-driving car algorithm building blocks. It was great fun to have it implemented! I feel there is some overlap of path planning with topics of Model Predictive Control, dicussed earlier. IMO, it would be great to have the simulator to contain a single project that combines those two topics.
