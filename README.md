# Baxter MoveIt Experiments

Set of realistic obstacle scenes for motion planning experiments on simulated Baxter Research Robot 
# Depends On
[ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

[Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)

[Baxter Simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)

[MoveIt!](https://moveit.ros.org/install/source/)

[Baxter MoveIt! Configuration](http://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)

# Setting Up Experiments
The main script, ```motion_planning_data_gen.py``` uses the MoveIt Python API for setting up the environment and creating motion plan requests. The program can be used with the default MoveIt OMPL motion planners as is. To use non-default OMPL planners with the Baxter MoveIt interface, this can be done by modifying the ```planning_context_manager.cpp``` file in the ```moveit_planners_ompl``` package to include the necessary OMPL headers and register the planner in the ```registerDefaultPlanners()``` function. Then in the ```baxter_moveit_config``` package, the file ```config/ompl_planning.yaml``` file can be modified to configure the planner and apply it as the default planner (using BIT* as an example):

```
planner_configs:
  BITStarkConfigDefault:
    type: geometric::BITstar
...
right_arm:
  default_planner_config: BITStarkConfigDefault
```

after making any of these changes rebuild your ROS workspace with ```catkin build```.

 The filename to save path data to should be configured in the ```main()``` loop of the Python program, 

```python
pathsFile = "data/path_data_example"
```

along with other experiment configuration such as MoveGroup planning timeout

```python
max_time = 300
group.set_planning_time(max_time)
```

or the condition for ending data collection (such as number of total planning attempts)
```python
while (total_paths < 30): #run until either desired number of total or feasible paths has been found
    ...
```

# Environments
The environment meta-data is saved in the pickled file ```env/trainEnvironments.pkl``` and the .STL files for the obstacles (book, soda can, mug, and bottle) are save in the ```meshes/``` directory. The environment data includes the dimensions, z-offset, workspace locations, and default mesh file path for loading the scene. A table planning scene interface is included in the script which loads this environment meta data and applies the different environments to the MoveIt scene such that the MoveIt collision checker and planner can be used with these obstacles in their respective locations. For each environment, there is also a set of collision-free configurations which resemble a grasp near the table surface saved in the pickle file ```env/trainEnvironments_testGoals.pkl``` which are similarly loaded in the main script to sample from when creating planning requests. 

# Running Experiments and Analyzing Data
The simulated robot and general MoveIt environment can be set up by launching
```
roslaunch baxter_moveit_experiments baxter_moveit.launch
```
and then the Python script ```motion_planning_data_gen.py``` can be run with a ROS node name as a single command line argument to set up the motion planning experiment with the various environments,
```
python motion_planning_data_gen.py test
```

The path planning data for each environment, including the paths, planning time, path cost (C-space euclidean length), and number of successful/total planning requests are recorded in a dictionary and periodically saved in the ```data/``` folder to be analyzed or played back on the robot. ```comparison.ipynb```  in ```analysis/``` and the ```playback_path.ipynb``` notebooks are simplified examples of using the saved planning data for data analysis or visualizing the paths on the robot using the Baxter interface (ensure the robot is enabled before playing back paths, with ```rosrun baxter_tools enable_robot.py -e``` in the terminal).
