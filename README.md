# Aim

The overarching objective entails the development and realization of an efficacious motion planning framework, designed to adeptly circumvent both stationary and moving impediments, all the while accurately adhering to the lane's central trajectory. Moreover, this comprehensive endeavor extends to encompass the seamless negotiation of stop signs. This multifaceted goal is attained through the integration of behavioral planning logic, inclusive of meticulous static collision assessment, judicious path determination, and the precise synthesis of a velocity profile on Ubuntu 22.04.

It is also a modified version of [the final project](https://www.coursera.org/learn/motion-planning-self-driving-cars).


# Implementation
In separate terminals:
```
opt/CarlaSimulator$ ./CarlaUE4.sh /Game/Maps/Course4 -windowed -carla-server -benchmark -fps=30
/opt/CarlaSimulator/PythonClient/Planning_YcK$ python3.6 module_7.py
```

![TP](https://github.com/Yigit-Kuyu/CARLA_Trajectory_Planning/blob/main/TrajectoryPlanning.PNG)
