# Path Planner for a Self Driving Car
This project was submitted as part of Udacity's Self Driving Car Nanodegree program. For more details see [Udacity's Self Driving Car Nanodegree](https://in.udacity.com/course/self-driving-car-engineer-nanodegree--nd013/?).
 
## Objective
The objective of this project is to safely drive a car around a virtual highway. The car should try to go as close as possible to the 50 MPH speed limit and pass traffic whenever it makes sense. The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Requirements
   
### Simulator.
This projects requires the Udacity provided simulator which can be found at (https://github.com/udacity/self-driving-car-sim/releases).


## Solution

### Overview
The following approach was used to meet the objectives - 

* The best lane for the target vehicle to drive is determined based on the score of each lane.
* A planning function then determines the next states of the car in order to reach the best lane.
* Feasibility is checked for the next state of the car, in order to avoid collisons or maintain comfort.
* If feasible, a trajectory is generated for the car to follow and move to next state. This is repeated for each of the states in order to reach the target lane. 
* Once the target lane is reached the loop continues, thus ensuring the target vehicle always attempts to stay on the best lane while driving.

#### Determination of Best Lane

The highway is made up of three lanes that the car could drive on. Lane 0 is closest to the double yellow line in the middle of the road. Lane 1 is the center lane while Lane 2 is the rightmost.  

The first step is to use a scoring function that rates each lane based on the current position of the car and the sensor fusion data that contains states of other cars. The following attributes were checked while determining the score - 
* The gap between the cars ahead and behind of our vehicle
* The speed of the car ahead of us in the lane
* The distance of the car ahead of us
* The distance of the car behind us

A slight priority is given to the current lane in order to avoid lane shifts if staying in the lane makes sense. This code can be found in main.cpp - 

```
main.cpp ln:379

double getLaneScore(int lane, double car_s, double currentSpeed, double prevPathSize, vector<vector<double>> sensorFusion) {

    double score = 0;
    // get the distance of the car ahead
    double frontDist = getNearestCarDistance(lane, car_s, prevPathSize, sensorFusion, true, 100);

    // get the distance of the car behind
    double rearDist = getNearestCarDistance(lane, car_s, prevPathSize, sensorFusion, false, 100);

    // get the free gap in the lane
    double laneGap = (car_s+frontDist) - (car_s-rearDist);

    // get the speed of the car ahead
    double frontSpeed = getFrontCarSpeed(lane, car_s, prevPathSize, sensorFusion, 100);

    // add score if target lane has better speed
    if (frontSpeed>currentSpeed) {
        score = score + (frontSpeed-currentSpeed) * SPEED_WEIGHT;
    }

    // add score based on the gap in the lane
    if (laneGap > 50) {
        score = score + laneGap * LANE_GAP_WEIGHT;
    }

    // add score based on the distance of the car ahead
    if (frontDist>50) {
        score = score + frontDist * FRONT_DISTANCE_WEIGHT;
    }

    // add score based on the distance of the car behind
    if (rearDist>20) {
        score = score + rearDist * REAR_DISTANCE_WEIGHT;
    }

    return score;
}
```

#### Future States of the Car

Our vehicle could have any of the following states -
* KL - Keep Lane and follow car ahead of us
* PCL/PCR - Keep lane and bring speed to match the car in the left/right lane in order to move over
* CL/CR - Change lane left or right and match speed of the car ahead in that lane

The second step is to get the possible future states of the car. The following are the different cases and the possible states - 

* **Current Lane is 0 and Target Lane is 1** PCR,CR
* **Current Lane is 0 and Target Lane is 0** KL
* **Current Lane is 2 and Target Lane is 0** PCL,CL

This is implemented in main.cpp at line 450 - 

```
main.cpp ln:450

vector<string> getStatesForTarget(int currentLane, int targetLane) {
    vector<string> states;

    // if lane is not valid or target is same as current lane return KL
    if (targetLane < 0 || targetLane > 2 || (currentLane==targetLane)) {
        states.push_back("KL");
        return states;
    }

    // if lane state is to move right set states as PCR or CR
    // else set states as PCL or CL
    if (currentLane-targetLane < 0) {
        states.push_back("PCR");
        states.push_back("CR");
    } else {
        states.push_back("PCL");
        states.push_back("CL");
    }

    return states;
}
```

#### Feasiblity of States

The next step is to check the most feasibile state from among the future states and determine its properties. The state properties contain the recommended lane and the speed to follow in order to execute that state. This step also ensures the car executes the states in the right sequence. For example, the car can never move from KL to CR without implementing the intermediate PCR state. Also while the car is at the CL state the only possible future states could be KL state as we do not want the car to immediately change lanes again. This is implemented in main.cpp at line 475 - 
```
main.cpp ln:475

State getStateProperties(int currentLane, double currentSpeed, string prevState, string targetState, double car_s, double prevPathSize, vector<vector<double>> sensorFusion) {

    // set default values for the state
    State result;
    result.speed = -1;
    result.lane = getStateLane(currentLane, targetState);
    result.label = targetState;

    // if target state is keep lane, set the speed property to speed of the car ahead, this should make the vehicle to follow the car
    if (targetState.compare("KL") == 0) {
        result.speed = getFrontCarSpeed(result.lane, car_s, prevPathSize, sensorFusion, 30);
    }

    // if target state is CL or CR
    else if (targetState.compare("CL") == 0 || targetState.compare("CR") == 0)  {
        // check if there are cars ahead in the target lane
        if (!isCloseAhead(result.lane, car_s, prevPathSize, sensorFusion, 20)) {
            // check if there are no cars at 30m distance behind
            if (!isCloseBehind(result.lane, car_s, prevPathSize, sensorFusion, 30)) {
                result.speed = SPEED_LIMIT;
            }
            // check if there are no cars at 15m distance behind
            else if (!isCloseBehind(result.lane, car_s, prevPathSize, sensorFusion, 15)) {
                // make a move only if the current speed is greater than the rear car speed
                double rearCarSpeed = getRearCarSpeed(result.lane, car_s, prevPathSize, sensorFusion, 50);
                if (currentSpeed - rearCarSpeed > 10) {
                    result.speed = SPEED_LIMIT;
                }
            }
        }
    }
    // if target state is PCL or PCR
    else if (targetState.compare("PCL") == 0 || targetState.compare("PCR") == 0) {
        // set speed to follow the car in current lane
        double currLaneSpeed = getFrontCarSpeed(currentLane, car_s, prevPathSize, sensorFusion, 30);
        result.speed = currLaneSpeed;
        // if state has been PCL or PCR before, reduce the speed in order to make a lane change
        if (prevState.compare("PCL") == 0 || prevState.compare("PCR") == 0) {
            if (currLaneSpeed > 45.0) {
                result.speed = 44.5;
            }
        }
        result.lane = currentLane;
    }

    return result;
}
```

#### Trajectory Generation

The next step is to generate a trajectory the car has to follow based on its current state. It should be noted that the car is always in any of the 5 states as mentioned above. The spline library (http://kluge.in-chemnitz.de/opensource/spline/) was used to make the path smooth and minimize jerk. After each iteration the left over paths were used in order to maintain continuity. The part that generates trajectory can be found at line 764 of main.cpp -
```
main.cpp ln:764

// configure the spline
tk::spline s;
s.set_points(ptsx, ptsy);

// add all previous points to the path
for(int i=0;i<prev_size;i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
}

// generate path for 30m ahead
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
double x_add_on = 0;
double N;
N = target_dist / (.02 * ref_vel/2.24);

// create points at comfortable distance and add all to previous path
for (int i=1; i<=50-prev_size;i++) {

    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);
    double x_ref = x_point;
    double y_ref = y_point;

    // remember the current point
    x_add_on = x_point;

    // get the points back to global co-ordinates
    x_point = ref_x + (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = ref_y + (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
}
```

This loop continues in order to ensure the vehicle always tries to maintain the best lane. Also an intentional delay was provided after each lane change in order to avoid jerk and constant lane fluctuations. The car was able to drive the entire lap (~7.3 miles) in about 10 minutes. The car was able to drive more than 22 miles without any incidents at which point I had to close the simulator as I had lost my patience.

## Improvements & Future Considerations

* Make our vehicle follow a car more smoothly. Currently, it fluctuates in speed as it tries to shift lanes. Ideally it should maintain a minimum distance and shift during opportunity.
* Implement emergency braking in order to avoid colliding with rogue cars. 

## Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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