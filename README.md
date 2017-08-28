# CarND-Controls-PID
[Self-Driving Car Engineer Nanodegree Program]

---
This project is to build a controller that adjust the steering and speed of an self-driving car automatically to drive on the road.

## Dependencies

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Logics

References: https://github.com/jeremy-shannon/CarND-PID-Control-Project.

In this project, I used two PID controllers to control the steering and the throttle of a simulated car based on the deviation it makes from the center of the lane. The discussion about PID on these two variables can be understood better through this [link](https://www.youtube.com/watch?v=4Y7zG48uHRo&feature=youtu.be). Note that the outputs from these PIDs must be in the range [-1,1]. However, tuning P, I, D gains is usually a challenge due to different turning scenarios on the road. 

In order to tune P, I and D gains, I used a common method called [Twiddle](https://www.youtube.com/watch?v=2uQ2BSzDvXs) explained by Sebastian Thrun. The algorithm optimizes one parameter at a time, checking if increasing or descreasing by a certain optimizing step helps to reduce the performance error. If the error is reduced, the algorithm increases the step of this parameter; otherwise it reduces the step. In either case, the algorithm will continue to the next parameter. The optimization will loop back once all parameters have been gone through.

## Implementation

In self-driving car, there is a challenge in evaluating the performance error before knowing if the new Twiddled parameter is effective. To measure performance, we need to let the car stable first (I picked 100 sampling steps) then measure the Summ of Squared Error track deviation through a period (I picked 2000 sample steps. Need to cover about 1 lap to fully understand the car performance in a whole lap) under src/PID.cpp.

I let my car runs for half an hour to tune the PID controllers and fixed the final gains under src/main.cpp.
