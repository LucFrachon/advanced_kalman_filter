# advanced_kalman_filter
Sensor fusion (LIDAR/RADAR) using an Extended Kalman Filter

This is Term 2 / Project 1 of the Self-Driving Car Nanodegree by Udacity.

## Directories and files

* `src ` contains the code (in C++).
    - `main.cpp`: main code that connects to the simulator, reads inputs from `../data/obj_pose-laser-radar-synthetic-input.txt`, 
    executes `ProcessMeasurement()` (see below) and calculates and ouputs the root mean squared errors (RMSE) of estimates 
    (calculated by the Kalman Filter) vs ground truth (provided in the data file). This code was entirely provided by Udacity.
    - `FusionEKF.cpp` / `FusionEKF.h`: define a Fusion Extended Kalman Filter class with the `ProcessMeasurement()` method, which 
    initilizes an instance of the `kalman_filter` class and calls the `Predict()` and `Update()` / `UpdateEKF()` methods.
    - `kalman_filter.cpp` / `kalman_filter.h`:  define the KalmaFilter class used in `FusionEKF::ProcessMeasurement()`. Its public
    variables are matrices used in the KF calculations. It has the following methods:
        - `Init()` that assigns initial values to the public variables
        - `Predict()` that runs the prediction step of the KF (assuming linear motion)
        - `Update()` / `UpdateEKF()` that run the update step of the KF for lidar and radar inputs respectively. The latter method
        calls `Tools::CalculateJacobian()` to approximate the non-linear radar measurements to a linear function using a 1st order
        Taylor development.
    - `tools.cpp` / `tools.h`: define the `Tools` class that contains two methods, `CalculateRMSE()` and `CalculateJacobian()`. The first 
    is called by `main.cpp` and the latter by `FusionEKF::ProcessMeasurement()`.
    - `measurement_package.h`: defines the `MeasurementPackage` class which contains each set of measurements along with its source 
    (radar / lidar) and its timestamp.

* `build` contains the build objects and the executable file `ExtendedKF`

* `data` contains the data file used in the main program. It simulates tracking  moving objects with both radar and lidar.

* `Docs` contains an explanation of the data format and instructions to generate data with Matlab

* The root directory contains files used to build the executable as well as two `.sh` files that automate the environment set up on Ubuntu 
and Mac systems. For Windows users, it is recommended that they use the Ubuntu BASH that is included in Windows 10 Creator's Update(make 
sure that it is updated to Ubuntu 16.04 LTS).

## How to use

* First, install the [Udacity Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* Start the simulator (this can be done in your Windows session if you are using the Windows 10 Ubuntu BASH)
* In the command line terminal (Ubuntu BASH if you are using Windows), navigate to your `build/` folder and execute
`ExtendedKF` by typing `./ExtendedKF`
* If everything is set up correctly, you should see a message saying "Listening to port 4567" and "Connected!!!"
* In the Simulator's main menu, make sure that "Projec 1/2" is selected and click on the "Select" button
* Click on the "Start" button to run the simulator. 
* The blue car represents the ground truth values, the red dots are the lidar measurements and the blue dots are the radar 
measurements (notice the arrow within each blue dotm it indicates the bearing of this particular reading)
* The estimated position as calculated by the EKF, will appear as a trace of green triangles.
* RMSE values are provided for the x and y positions as well as the x and y components of the velocity vector. The lower those
values, the more accurate the EKF.



