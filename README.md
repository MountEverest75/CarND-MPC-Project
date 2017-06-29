# CarND-Controls-MPC
## Self-Driving Car Engineer Nanodegree Program
---
## Overview
The primary goal of this project is to implement a Model Predictive Controller that could steer and successfully navigate a car in the simulator by sending steering and acceleration/decceleration commands. The project uses Ipopt and CppAD libraries to calculate trajectory and actuations to minimize error using third degree polynomial fit to the given waypoints. The solution has been optimized based on vehicle kinematic model and cost function based on Cross Track Error(CTE) and Orientation Error(epsi).

---

## Model
The MPC model involves tracking state and actuation commands using the attributes given below:

The state is calculated using the following attributes:
* Vehicle position denoted by x and y
* Vehicle orientation denoted by psi
* Vehicle velocity by v
* Vehicle Cross Track Error by cte
* Vehicle Orientation Error by epsi

The actuation attributes are:
* Steering angle denoted by delta
* Acceleration denoted by "a"

The model described combines and state and actuation commands from previous time-step to calculate the current time-step using the equations given below:

![](./images/model_equations.png)

---

## Tuning - Time-step and Elapsed duration
The final values chosen for time-step length N and duration dt are 10 and 0.1 respectively. The values have been arrived by reviewing Udacity discussion forums and Udacity Q&A sessions on youtube. The combination of N=10 and dt=0.1 has been found to optimal focusing on a time lapse of one second to find MPC trajectory. Anything long or short has been observed to have adverse effect on the performance of the car on the track. The combinations tried are given below:

  * N = 25 and dt = 0.05
  * N = 15 and dt = 0.02
  * N = 10 and dt = 0.1  

Larger N values have been found to increase processing time in addition to latency effecting the reaction time on the track. A value of time-step duration that syncs up with latency of 100ms (100/1000=0.1s) has been found to be optimal value.

---

## MPC Preprocessing and Polynomial Fitting
The waypoints are transformed to car's perspective by considering vehicles co-ordinates(px, py) and orientation(psi) to be zero simplifying third degree polynomial fit. This approach helped in translating reference path relative to car's co-ordinates subsequently applying the coefficients for MPC calculation.

---

## Latency
Since the actuations are applied to subsequent time-steps the original equations from the lessons have to be tuned to account for the latency from previous time-steps. This change has been demonstrated lines 107-110 in MPC.cpp.

---
## Additional Tuning
At higher speeds i.e. when the car goes at more than 35mph, the car has been found to react to high values to steering. The cost function had to be tuned to support higher speeds. The following factors have been chosen to ensure proper balance. The multiplication factors have been decided with trial and error. The Udacity Q&A session provided some clues on what values would ensure car goes round the track successfully at speeds of 70mph and more. The code lines can be found in MPC.cpp lines 51-70.
* Reference State - Changes related to following parameters
  *  Cross Track Error (cte) - Multiplier of 3000
  *  Orientation Angle (psi) - Multiplier of 3000
  *  Velocity (v)
* Actuations (Changes in Steering angle)
  *  Changes in Steering Angle (Delta) - Multiplier of 200
  *  Acceleration (a) - Multiplier of 10

---

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
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
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
