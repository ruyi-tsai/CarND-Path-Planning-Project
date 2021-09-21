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
### Prediction
The prediction component estimates path in the furture.We assume three parameter to check collision in the future.
```
bool Car_Collision = false;
bool Car_left_close = false;
bool Car_right_close = false;
```
Preiction flag(Car_Collision,Car_left_close,Car_right_close) according to the sensor fusion data.
In the behavior planner as shown in the code below, it check for the Car_Collision flag set to true.
And then will decision to turn left or turn right.
```
 if(Car_Collision)
            {
              std::cout <<"lane="<<lane<< "\n";
              std::cout <<"Car_left_close="<<Car_left_close<< "\n";
              std::cout <<"Car_right_close="<<Car_right_close<< "\n";
            	if(lane==car_middle)
                {
                	if(!Car_left_close) lane = car_left;
                    else if(!Car_right_close) lane = car_right;
                    else ref_vel -= speed_diff;
                }
                else if(lane==car_right)
                {
                	if(!Car_left_close) lane = car_middle;
                    else ref_vel -= speed_diff;
                }
               else if(lane==car_left)
               {
                 	if(!Car_right_close) lane = car_middle;
                    else ref_vel -= speed_diff;
               }
             
            }
            else if(ref_vel < max_accel)
            {
            	ref_vel += speed_diff;
            }
```

### Trajectory planning
Based on the desired immediate behavior, the trajectory planning component will determine which trajectory is best for executing this behavior.
For trajectory generation, we are using spline instead of polynomial trajectory generation.
```
 tk::spline s;
 s.set_points(ptsx, ptsy);
```
We add previous path .
```
ref_x = previous_path_x[prev_size-1];
ref_y = previous_path_y[prev_size-1];
double prev_car_x = previous_path_x[prev_size-2];
double prev_car_y = previous_path_y[prev_size-2];
```
Now we need to add 3 future points to ptsx, psy vecotrs.
```
vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
```
Now we need to find the all spline points till the horizon(say 30m) value so that spacing the way that ego car can travel at desired speed. 
```
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(target_x*target_x + target_y*target_y);
double x_add_on = 0;
```
We can calculate the spline points .
```
  for ( int i = 1; i < 50-prev_size; i++ ) {
             double N = target_dist/(0.02*ref_vel/2.24);
             double x_point = x_add_on + target_x/N;
             double y_point = s(x_point);
             x_add_on = x_point;
             double x_ref = x_point;
             double y_ref = y_point;
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }
```
