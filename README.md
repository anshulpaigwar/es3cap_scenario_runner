ScenarioRunner for CARLA
========================
This repository contains traffic scenario definition and an execution engine
for CARLA. It also allows  the execution of a simulation of the CARLA Challenge.
You can use this system to prepare your agent for the CARLA Challenge.

Usage
----------
First Run Carla server with Town 03
````
DISPLAY= ./CarlaUE4.sh -benchmark -fps=10
````

Run roscore and then risk_grid_validation.py it launches cmcdot and carla_ros_bridge with cmcdot client 
```
python risk_grid_validation.py
```


License
-------

ScenarioRunner specific code is distributed under MIT License.
