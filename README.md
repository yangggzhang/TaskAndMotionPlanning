# TaskAndMotionPlanning

Implementation of [Combined Task and Motion Planning Through an Extensible Planner-Independent Interface Layer](https://people.eecs.berkeley.edu/~russell/papers/icra14-planrob.pdf)
This work is implemented based on [MoveIt! interface](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

Example demo:
Terminal 1:
```
roslaunch panda_moveit_config demo.launch
```
Terminal 2:
```
roslaunch planner scene1.launch
roslaunch planner scene2.launch
roslaunch planner scene3.launch
```
