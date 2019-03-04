import numpy as np
from numpy import matlib
import rospy
import baxter_interface
from moveit_msgs.msg import RobotState, DisplayRobotState, PlanningScene, RobotTrajectory, ObjectColor
from moveit_commander import PlanningSceneInterface, RobotCommander, MoveGroupCommander, MoveItCommanderException
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point
from std_msgs.msg import Header
import random
import time
import sys
import tf
import os
import pickle

from util.get_state_validity import StateValidity
from util.moveit_functions import create_scene_obs, moveit_scrambler, moveit_unscrambler, set_environment

class TableSceneModifier():
    def __init__(self, mesh_path='./meshes/'):
        # Workspaces boundaries manually set up to have Baxter reach toward the table as if grasping an object
        # Can use this to create new target configurations for the robot

        #x, y boundary 
        self.valid_pose_boundary_x = [[0.8, 0.95], [-0.2, 1.02]]
        self.valid_pose_boundary_y = [[-0.69, 0.0], [-1.05, -0.79]] #top shelf [xmin, xmax, ymin, ymax, z1 = 0.24, z2 = 0.78

        self.boundary_dict = {}
        self.z = 0.24
        self.z_range = 0.12

        self.obstacles = {}
        self.mesh_path = mesh_path

        self.setup_boundaries()
        self.setup_obstacles()

    def setup_boundaries(self):
        self.boundary_dict['x_bounds'] = self.valid_pose_boundary_x
        self.boundary_dict['y_bounds'] = self.valid_pose_boundary_y

        self.boundary_dict['x_range'] = []
        self.boundary_dict['y_range'] = []
        for area in range(len(self.valid_pose_boundary_x)):
            self.boundary_dict['x_range'].insert(area, max(self.valid_pose_boundary_x[area]) - min(self.valid_pose_boundary_x[area]))
            self.boundary_dict['y_range'].insert(area, max(self.valid_pose_boundary_y[area]) - min(self.valid_pose_boundary_y[area]))

    def setup_obstacles(self):
        # names = ['mpnet_box1', 'mpnet_box2', 'mpnet_box3', 'mpnet_box4', 'mpnet_box5'] #5 boxes
        # obstacles{}.keys() = ['name', 'dimensions', 'is_mesh', 'mesh_file', 'orientation', 'z_offset']
        obstacles = [
        ('mpnet_box1', [0.14, 0.09, 0.12], 0, None, None, 0.02),
        ('mpnet_bottle', [0.001, 0.001, 0.001], 1, self.mesh_path+'bottle.stl', None, -0.02),
        ('mpnet_pepsi', [0.001, 0.001, 0.001], 1, self.mesh_path+'pepsi.STL', [0.70710678, 0, 0, 0.70710678], -0.041),
        ('mpnet_coffee', [0.035, 0.035, 0.035], 1, self.mesh_path+'coffee.stl', [0.70710678, 0, 0, 0.70710678], -0.035),
        ('mpnet_book', [0.0017, 0.0017, 0.0017], 1, self.mesh_path+'Dictionary.STL', None, -0.04)
        ] 

        for obs in obstacles:
            self.obstacles[obs[0]] = create_scene_obs(obs[0], obs[1], obs[2], obs[3], obs[4], obs[5])

    def get_permutation(self):
        area = 0 if (np.random.random() < 0.5) else 1
        object_location_x = np.random.random() * (self.boundary_dict['x_range'][area]) + min(self.boundary_dict['x_bounds'][area])
        object_location_y = np.random.random() * (self.boundary_dict['y_range'][area]) + min(self.boundary_dict['y_bounds'][area])
        # object_location_z = self.z[0] if (np.random.random() < 0.5) else self.z[1] #removed when not using shelf
        object_location_z = self.z

        pose = [object_location_x, object_location_y, object_location_z]

        return pose

    def permute_obstacles(self):
        pose_dict = {}
        for name in self.obstacles.keys():
            pose_dict[name] = self.get_permutation()

        return pose_dict


class PlanningSceneModifier():
    def __init__(self, obstacles):
        self._obstacles = obstacles

        self._scene = None
        self._robot = None

    def setup_scene(self, scene, robot, group):
        self._scene = scene
        self._robot = robot
        self._group = group

    def permute_obstacles(self, pose_dict):
        for name in pose_dict.keys():
            pose = PoseStamped()
            pose.header.frame_id = self._robot.get_planning_frame()
            pose.pose.position.x = pose_dict[name][0]
            pose.pose.position.y = pose_dict[name][1]
            pose.pose.position.z = pose_dict[name][2] + self._obstacles[name]['z_offset']

            if self._obstacles[name]['orientation'] is not None:
                pose.pose.orientation.x = self._obstacles[name]['orientation'][0]
                pose.pose.orientation.y = self._obstacles[name]['orientation'][1]
                pose.pose.orientation.z = self._obstacles[name]['orientation'][2]
                pose.pose.orientation.w = self._obstacles[name]['orientation'][3]

            if self._obstacles[name]['is_mesh']:
                print("Loading mesh from: " + self._obstacles[name]['mesh_file'])
                self._scene.add_mesh(name, pose, filename=self._obstacles[name]['mesh_file'], size=self._obstacles[name]['dim'])
            else:
                self._scene.add_box(name, pose, size=self._obstacles[name]['dim'])

        rospy.sleep(1)
        print("Objects in the scene: ")
        print(self._scene.get_known_object_names())

    def delete_obstacles(self):
        #scene.remove_world_object("table_center")
        for name in self._obstacles.keys():
            self._scene.remove_world_object(name)

def compute_cost(path):
    state_dists = []
    for i in range(len(path) - 1):
        dist = 0
        for j in range(7): #baxter DOF
            diff = path[i][j] - path[i+1][j]
            dist += diff*diff

        state_dists.append(np.sqrt(dist))
    total_cost = sum(state_dists)
    return total_cost

def main():
    rospy.init_node(sys.argv[1])

    # sometimes takes a few tries to connect to robot arm 
    limb_init = False
    while not limb_init:
        try:
            limb = baxter_interface.Limb('right')
            limb_init = True 
        except OSError:
            limb_init = False 

    neutral_start = limb.joint_angles()
    min_goal_cost_threshold = 2.0


    # Set up planning scene and Move Group objects
    scene = PlanningSceneInterface()
    scene._scene_pub = rospy.Publisher(
        'planning_scene', PlanningScene, queue_size=0)
    robot = RobotCommander()
    group = MoveGroupCommander("right_arm")
    sv = StateValidity()

    set_environment(robot, scene)

    # Setup tables (geometry and location defined in moveit_functions.py, set_environment function)
    # set_environment(robot, scene)

    # Additional Move group setup

    # group.set_goal_joint_tolerance(0.001)
    # group.set_max_velocity_scaling_factor(0.7)
    # group.set_max_acceleration_scaling_factor(0.1)
    max_time = 300
    group.set_planning_time(max_time)


    # Dictionary to save path data, and filename to save the data to in the end
    pathsDict = {}
    pathsFile = "data/path_data_example_quick"

    # load data from environment files for obstacle locations and collision free goal poses
    with open("env/trainEnvironments.pkl", "rb") as env_f:
        envDict = pickle.load(env_f)

    with open("env/trainEnvironments_testGoals.pkl", "rb") as goal_f:
        goalDict = pickle.load(goal_f)

    # Obstacle data stored in environment dictionary, loaded into scene modifier to apply obstacle scenes to MoveIt
    sceneModifier = PlanningSceneModifier(envDict['obsData'])
    sceneModifier.setup_scene(scene, robot, group)

    robot_state = robot.get_current_state()
    rs_man = RobotState()
    rs_man.joint_state.name = robot_state.joint_state.name


    # Here we go
    test_envs = envDict['poses'].keys() #slice this if you want only some environments
    done = False
    iter_num = 0
    print("Testing envs: ")
    print(test_envs)

    while(not rospy.is_shutdown() and not done):
        for i_env, env_name in enumerate(test_envs):
            print("env iteration number: " + str(i_env))
            print("env name: " + str(env_name))

            sceneModifier.delete_obstacles()
            new_pose = envDict['poses'][env_name]
            sceneModifier.permute_obstacles(new_pose)
            print("Loaded new pose and permuted obstacles\n")

            pathsDict[env_name] = {}
            pathsDict[env_name]['paths'] = []
            pathsDict[env_name]['costs'] = []
            pathsDict[env_name]['times'] = []
            pathsDict[env_name]['total'] = 0
            pathsDict[env_name]['feasible'] = 0

            collision_free_goals = goalDict[env_name]['Joints']
            
            total_paths = 0
            feasible_paths = 0
            i_path = 0

            while (total_paths < 30): #run until either desired number of total or feasible paths has been found

                #do planning and save data

                # some invalid goal states found their way into the goal dataset, check to ensure goals are "reaching" poses above the table
                # by only taking goals which have a straight path cost above a threshold
                valid_goal = False
                while not valid_goal:
                    goal = collision_free_goals[np.random.randint(0, len(collision_free_goals))]
                    optimal_path = [neutral_start.values(), goal.values()]
                    optimal_cost = compute_cost(optimal_path) 

                    if optimal_cost > min_goal_cost_threshold:
                        valid_goal = True 

                
                print("FP: " + str(feasible_paths))
                print("TP: " + str(total_paths))
                total_paths += 1
                i_path += 1
                   
                # Uncomment below if using a start state different than the robot current state
    
                # filler_robot_state = list(robot_state.joint_state.position) #not sure if I need this stuff
                # filler_robot_state[10:17] = moveit_scrambler(start.values())
                # rs_man.joint_state.position = tuple(filler_robot_state)
                # group.set_start_state(rs_man)   # set start

                group.set_start_state_to_current_state()

                group.clear_pose_targets()
                try:
                    group.set_joint_value_target(moveit_scrambler(goal.values())) # set target
                except MoveItCommanderException as e:
                    print(e)
                    continue

                start_t = time.time()
                plan = group.plan()

                pos = [plan.joint_trajectory.points[i].positions for i in range(len(plan.joint_trajectory.points))]
                if pos != []:
                    pos = np.asarray(pos)
                    cost = compute_cost(pos)
                    t = time.time() - start_t
                    print("Time: " + str(t))
                    print("Cost: " + str(cost))

                    # Uncomment below if using max time as criteria for failure
                    if (t > (max_time*0.99)):
                        print("Reached max time...")
                        continue  

                    feasible_paths += 1

                    pathsDict[env_name]['paths'].append(pos)
                    pathsDict[env_name]['costs'].append(cost)
                    pathsDict[env_name]['times'].append(t)
                    pathsDict[env_name]['feasible'] = feasible_paths
                    pathsDict[env_name]['total'] = total_paths
                    

                    # Uncomment below if you want to overwrite data on each new feasible path
                    with open (pathsFile + "_" + env_name + ".pkl", "wb") as path_f:
                        pickle.dump(pathsDict[env_name], path_f)



                print("\n")
               

            sceneModifier.delete_obstacles()
            iter_num += 1

            print("Env: " + str(env_name))
            print("Feasible Paths: " + str(feasible_paths))
            print("Total Paths: " + str(total_paths))
            print("\n")

            pathsDict[env_name]['total'] = total_paths
            pathsDict[env_name]['feasible'] = feasible_paths

            with open(pathsFile + "_" + env_name + ".pkl", "wb") as path_f:
                pickle.dump(pathsDict[env_name], path_f)

        print("Done iterating, saving all data and exiting...\n\n\n")

        with open(pathsFile + ".pkl", "wb") as path_f:
            pickle.dump(pathsDict, path_f)

        done = True

if __name__ == '__main__':
    main()
