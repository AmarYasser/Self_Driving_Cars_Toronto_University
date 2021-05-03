# Vehicle Control on CARLA Simualtor
This repo is an implementation of a vehicle controller for the CARLA simulator. 
The goal is to control the vehicle to follow a race track by navigating through preset waypoints. The vehicle needs to reach these waypoints at certain desired speeds, so both longitudinal and lateral control will be required.


## longitudinal control
The vehicle expects a reference velocity at each waypoint
For the  cruise control, we are using classical PID controller to get the desired acceleration based on the feedback states 
as shown in the next two figures:

![Vehicle_PID](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/PID_Vehicle.jpg "Vehicle_PID")

![Controller_Equation](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/PID.png "Controller_Equation")

We will only consider a high level controller and consdier a the output (Desired accerlation/Deccelration) direcly to throttle and break.
the low level controller is based on the engine parameters, this will be the next step :)

![Controller_Equation](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/Screenshot%20from%202021-05-03%2022-59-30.png)


## lateral control
This will be based on _Stanley_ Method, The Stanley method is the path tracking approach used by Stanford University’s autonomous vehicle entry in the DARPA Grand Challenge, Stanley.

![stanley Geometry](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/stanley_Geometry.png "stanley Geometry") 

![stanley_Equation](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/stanley_Equation.png "stanley_Equation")


## Runnig the simualtor 

- First be sure to install CARLA simulator on you machine

- Clone this repo inside `./PythonClient` Foldar inside the main Carla's directory
- Run the next command inside the main Caral's directory
    ```
    ./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30
    ```
- Then Run the following command inside our repo.
    ```
    python3 ./module_7.py
    ```

## Results

The Following pics shows the output of the controller (Steer, Throttle, Forward speed which should follow the reference one ant each time step)
![trajectory](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/trajectory.png)
![throttle_output](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/throttle_output.png)
![forward_speed](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/forward_speed.png)
![steer_output](https://github.com/AmarYasser/Self_Driving_Cars_Toronto_University/blob/main/01_Introction_to_self_driving_cars/Vehicle_Control_Project/controller_output/steer_output.png)


Next video shows the simulator 


[![CARLA simulator](http://img.youtube.com/vi/Pf4eKwxr598/0.jpg)](http://www.youtube.com/watch?v=Pf4eKwxr598 "CARLA simulator")

