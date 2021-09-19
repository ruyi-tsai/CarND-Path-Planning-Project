# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

## Rubic Points
### Trajectory Generator
1. Major Waypoints

First, given the lane I’m targeting to end up in, I generate 3 major path waypoints 50 meters away from each other. I also create 2 points very close to my position which reflect the current orientation of my car. More on why I did this bellow…

```
  vector<double> xy_path0 = getXY(car_s,6,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> xy_path1 = getXY(car_s+50*dist_inc,6,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> xy_path2 = getXY(car_s+50*2*dist_inc,6,map_waypoints_s,map_waypoints_x,map_waypoints_y);
```
2. Spline Generation

To know how I’m going to reach one point after another, I create a spline which runs through these major waypoints. A spline is a polynomial regression that goes exactly through each and every points that are given to it. Since I create these 2 points where my car is at, the generated spline will be tangent to my current trajectory and won’t create too much jerk or acceleration.
```
tk::spline s;
ptx.push_back(xy_path0[0]);
pty.push_back(xy_path0[1]);
ptx.push_back(xy_path1[0]);
pty.push_back(xy_path1[1]);
ptx.push_back(xy_path2[0]);
pty.push_back(xy_path2[1]);
s.set_points(ptx,pty);
```


3. Avoid collision
For avoid collision , I need use sensor_fusion to prediction other car position.If other cars may close (<30 mile). I need reduce speed.
```
 check_car_s +=double(pre_size*.02*check_car_speed);
              if(check_car_s>car_s && (check_car_s-car_s)>30)
              {
                 isclose = true;
                dist_inc = 0.2;
              }
```

     


