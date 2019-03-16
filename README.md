# CarND-Path-Planning-Project

This is the project submission for the highway path planning project for the Udacity Self-Driving Car Nanodegree.

## Approach

The goal of the project is to implement a path planning algorithm for a highway scenario. The project comes with the following:

1. A pre-built highway simulator built on top of the Unity gaming engine and a list of waypoints around the scene. The scene is a 7km stretch of highway with 3 lanes on each side. The simulator is populated with traffic driving roughly within 40-60 mph.
2. Localization data (and vehicle state) for the ego vehicle.
3. Sensor fusion data from on-board sensors.
4. A list of unconsumed points along the path, for each time step.

There are three main parts to a highway planning algorithm.

1. __Prediction:__ Understanding the environment around the ego vehicle using sensor fusion data, and how the environment will evolve over the planning horizon. In this project, prediction is accomplished by identifying nearby vehicles that could be potential collisions. See the first half of the `transition_state` function in [`planner.h`](https://github.com/anandraja13/CarND-Path-Planning-Project/blob/master/src/planner.h). Any vehicles within some distance of the ego vehicle on either side of the lane is flagged here so as to influence the behavior planning.  
2. __Behavior Planning:__ Using prediction and current state of the vehicle to decide on a collision-free, driveable and goal-oriented behavior. A rudimentary behavior planner is used (lower half of `transition_state` function), based on the following logic:

  * __Lane Keep:__ If no vehicle is in the ego lane and ahead of the ego vehicle, continue going straight and accelerating till we reach the speed limit.
  * __Lane Change:__ If a vehicle is in the ego lane and ahead of the ego vehicle, try changing lanes. First try changing to the left lane, and if that is not possible, try changing to the right lane. In order to do this, safety of the lane change maneuver is calculated by projecting forward the Frenet `s` coordinates of the vehicle in time and comparing it to the ego vehicle. The vehicle slows down while trying to change lanes to ensure smoothness of the trajectories.

3. __Trajectory Generation:__ Generating a trajectory to implement the behavior recommended by the behavior planning module. In this project, a piecewise Spline is generated in local Frenet coordinates. To ensure smoothness of the generated spline, the spline is generated using anchor points from the previous path and anchor points projected some distance forward. [This spline library](https://kluge.in-chemnitz.de/opensource/spline/) was used.

The end result is a set of waypoints that are fed to a controller.

## Meeting Project Specifications

* The code compiles correctly.

```
nands-MacBook-Pro:build Anand$ cmake .. && make
-- Configuring done
-- Generating done
-- Build files have been written to: /Users/Anand/udacity_car/term3/CarND-Path-Planning-Project/build
Scanning dependencies of target path_planning
[ 50%] Building CXX object CMakeFiles/path_planning.dir/src/main.cpp.o
[100%] Linking CXX executable path_planning
ld: warning: directory not found for option '-L/usr/local/Cellar/libuv/1.11.0/lib'
[100%] Built target path_planning
```

## Incremental Progress

* Set up repo, get the stationary car in the scene.
* Get the car to move in a straight line.
* Add a TrajectoryPlanner class to handle the planning.
* Get the car to follow the middle lane.
* Get the car to slow down to a pre-determined velocity when there's a car in front.
* Make the car obey acceleration/deceleration limits.
* Make the car change lanes using some rudimentary logic - however, jerk and accel limits are violated at turns.
* Obey acceleration and jerk limits - but drives very conservatively.
* Refactor behavior planning and update README.

![alt text](https://github.com/anandraja13/CarND-Path-Planning-Project/blob/master/images/5%20miles%20incident-free.png)
![alt text](https://github.com/anandraja13/CarND-Path-Planning-Project/blob/master/images/changing%20lanes.png)