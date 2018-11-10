# **Extended Kalman Filters** 

## Writeup Template


**Extended Kalman Filters Project**

In this project I am going to implement the extended Kalman filter in C++. Using a dataset for a simulated lidar and radar measurements detecting a bicycle that travels around your vehicle. I will use a Kalman filter c++ code, lidar measurements and radar measurements dataset to track the bicycle's position and velocity.

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles. The [video]() shows what the simulator looks like when a c++ script is using its Kalman filter to track the object. The simulator provides the script the measured data (either lidar or radar), and the script feeds back the measured estimation marker, and RMSE values from its Kalman filter.

## Environment setup
* The first step is to download the [Udacity Term 2 simulator](https://github.com/udacity/self-driving-car-sim/releases/)
* Install visual studio 2017 community edition
* Install visual studio [environment](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio)

## Running Environment

* Open the simulator. In the main menu screen select Project 1: Bicycle tracker with EKF.
* Once the scene is loaded you can hit the START button to observe how the object moves and how measurement markers are positioned in the data set. Also for more experimentation, "Data set 2" is included which is a reversed version of "Data set 1", also the second data set starts with a radar measurement where the first data set starts with a lidar measurement. At any time you can press the PAUSE button, to pause the scene or hit the RESTART button to reset the scene. Also the ARROW KEYS can be used to move the camera around, and the top left ZOOM IN/OUT buttons can be used to focus the camera. Pressing the ESCAPE KEY returns to the simulator main menu.
* run the project in visual studio

## The three main steps for programming a Kalman filter:
* initializing Kalman filter variables
* predicting where our object is going to be after a time step \Delta{t}Δt
* updating where our object is based on sensor measurements