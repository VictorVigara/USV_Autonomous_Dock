import enum
import time
import traceback
from collections import deque
from typing import Optional, Tuple
import json

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

import cv2 as cv

from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from skimage.transform import (hough_line, hough_line_peaks)

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, Point, Vector3
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Imu
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2 #used to read points 
from std_msgs.msg import Int16, Bool, Float32

import math

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf_transformations import euler_from_quaternion

from mbzirc_interfaces.action import Dock

from mbzirc_dock.approach_policy import ApproachPolicy
from mbzirc_dock.line_detection import detect_lines, filter_lines, est_line, all_points_valid, distance_point_to_line
from mbzirc_dock.orbit_policy import OrbitPolicy
from mbzirc_dock.policy import Policy, PolicyAction, PolicyState, PolicyCanceledException, PolicyOutOfBoundsException
from mbzirc_dock.stop_policy import StopPolicy
from mbzirc_dock.target_policy import TargetPolicy
from mbzirc_dock.touch_policy import TouchPolicy
from mbzirc_dock.tracker import Tracker2D
from mbzirc_dock.line import Line


class DockStage(enum.Enum):
    ON_SHORE = enum.auto()                  # USV starting state
    LEAVE_GATE = enum.auto()                # Go straight few meters to leave the gate
    TURNING_TO_POI = enum.auto()            # USV facing to POI coordinates
    STRAIGHT_TO_POI_BLIND = enum.auto()     # USV going straight forward to the POI without POI detection
    STRAIGHT_TO_POI_CAMERA = enum.auto()    # USV going straight forward to the camera detected POI
    STRAIGHT_TO_POI_LIDAR = enum.auto()     # USV going straight forward to the lidar detected POI
    WAIT_OBSTACLE = enum.auto()             # USV waiting because an obstacle is in the path
    ENTER_ORBIT = enum.auto()               # USV entering the POI orbit
    ORBIT = enum.auto()                     # USV orbitting the POI to find the largest side
    APPROACH = enum.auto()                  # USV approaching the POI
    TOUCH = enum.auto()                     # USV touching the POI
    BACK_TO_SHORE = enum.auto()             # USV coming back to shore
    STOP = enum.auto()                      # USV stopped            

class DockActionServer(Node):
    BASE_LINK = 'world'
    DT = 0.01

    def __init__(self):
        super().__init__('dock_action_server')

        # Run mode
        self.demo = False # Set TRUE if docking at pier

        # debug
        self.step_counter = 0
        self.debug = False
        self.predicted_points = []
        self.detected_points = []
        self.updated_points = []

        # Define colours to plot clusters
        self.cluster_colors = [[255.0, 0.0, 0.0], 
                          [0.0, 255.0, 0.0], 
                          [0.0, 0.0, 255.0], 
                          [255.0, 255.0, 0.0], 
                          [0.0, 255.0, 255.0], 
                          [255.0, 0.0, 255.0],
                          [255.0, 0.0, 0.0], 
                          [0.0, 255.0, 0.0], 
                          [0.0, 0.0, 255.0], 
                          [255.0, 255.0, 0.0], 
                          [0.0, 255.0, 255.0], 
                          [255.0, 0.0, 255.0],
                          ]
        
        # States dict
        self.USV_state_dict  = {'ON_SHORE': 0,          # USV starting state 
                                'TURNING_TO_POI': 1,    # USV facing to POI coordinates
                                'STRAIGHT_TO_POI': 2,   # USV going straight forward to the POI
                                'WAIT_OBSTACLE': 3,     # USV waiting because an obstacle is in the path
                                'ENTER_ORBIT': 4,       # USV entering the POI orbit
                                'ORBIT': 5,             # USV orbitting the POI to find the largest side    
                                'APPROACH': 6,          # USV approaching the POI
                                'TOUCH': 7,             # USV touching the POI
                                'BACK_TO_SHORE': 8,     # USV coming back to shore
                                'STOP': 9}              # USV stopped
        
        ###
        ### PARAMETERS
        ###

        # LEAVE GATE 
        self.leave_gate_vel = 1.0       # Velocity to leave the gate (m/s)

        # TURNING TO POI 
        self.turning_threshold = 3      # Threshold within turning angle reached to start next stage (degrees)

        # STRAIGHT TO POI BLIND
        self.straight_to_poi_blind_vel = 1.0    # Velocity to go straight to POI until camera detects de POI

        # STRAIGHT TO POI CAMERA
        self.straight_to_poi_camera_vel = 1.0   # Velocity to go straight to POI when camera detects de POI

        # STRAIGHT TO POI LIDAR
        self.match_POI_cam_lid_angle = 4        # Angle range to match camera and lidar detection
        
        # DBSCAN clustering parameters
        self.epsilon = 2
        self.min_samples = 1
        self.lidar_cluster_min_points = 5

        ## Line detection parameters
        self.grid_resolution = 0.1       # Resolution of the grid (adjust based on your needs)
        self.dilate_kernel_size = 1      # Kernel size to apply dilate to laser scan image to detect lines
        self.angle_difference = 10       # Angle difference between previous best_line detected and the current one
                                         # to avoid selecting a different best line detected that is not the target
        self.points_in_line_th_1 = 0.2   # Threshold to select points that belong to the first line detected
        self.points_in_line_th_2 = 0.05  # Threshold to select points that belong to the final line


        # Object avoidance parameters
        self.clear_path_width = 6.0
        self.clear_path_length = 50.0

        # Parameter to match the target from the initial position given POI coords
        self.distance_threshold = 1
        self.ocluded_target_threshold = 1 # Match the last target tracked pos ocluded with a cluster

        # parameters crop pc2
        self.x_crop = 2.2
        self.y_crop = 1.1

        # parameters for the orbit
        self.r_orbit = 10.0
        self.v_orbit = 2.0

        # parameters for the line detection
        # NOT IN USE
        self.line_thres_min = 0.7
        self.line_thres_max = 1
        self.angle_threshold = np.deg2rad(50) # 150
        self.prev_angle = None

        # Parameters to discard outlier measurements
        # NOT IN USE
        self.window_size = 20
        self.th_factor = 2

        # parameters for line tracking
        self.P0 = 1.0
        self.R0 = 0.1
        self.Q0 = 0.5
        self.mode = 'p'
        self.tracked_start: Optional[Tracker2D] = None
        self.tracked_end: Optional[Tracker2D] = None

        # parameters for the approach policy
        self.trajectory_relative = [
            np.array([[-8.0,  6.0]]).T,
            np.array([[-6.0,  5.0]]).T,
            np.array([[-4.0,  4.0]]).T,
            np.array([[-2.0,  3.5]]).T,
            np.array([[ 0.0,  3.5]]).T,
            np.array([[ 2.0,  3.5]]).T,
        ]

        self.trajectory_speeds = [
            2.0,
            2.0,
            2.0,
            2.0,
            1.0,
            0.5
        ]

        # parameters for the touch policy
        self.usv_width = 3.4
        self.v_touch = 0.5
        self.touch_precision = 0.3


        ### 
        ### INITIALIZE VARIABLES
        ###

        # TURNING TO POI variables
        self.USV_control_msg_sent = False

        # Lidar subscriber
        self.lidar_msg_received = False

        # Lidar target detected
        self.POI_lidar_detected = False

        # Target detected flag
        self.target_detected = False        # True if target detected in last measurement

        # Object trackers array
        self.trackers_objects = []          # Position tracker for detected objects
        self.tracked_objects_points = []    # Stores the points from the tracked objects
        self.tracker_target = None          # Tracker for the target vessel
        self.tracked_target_points = None   # Pointcloud points from the target vessel

        # Colision variables
        self.collision_objects = []         # Boolean array 1: obstacle in the path, 0: obstacle outside the path
        self.colision_status = False        # False -> No colision, True -> Obstacles in the path

        # Best line to dock the target vessel
        self.best_line = None

        # Initial target coordinates
        self.POI_initial_coords = None
        self.POI_initial_received = False

        # USV feedback
        self.yaw_USV = None     
        self.yaw_USV_received = False       

        # Camera target coordinates
        self.POI_camera_coords = None
        self.POI_camera_received = False
        self.POI_camera_angle = None
        
        # Initialize trajectory targets
        self.trajectory_targets = [
            np.array([[0.0, 0.0]]).T for _ in range(len(self.trajectory_relative))
        ]
        
        # USV mission state
        self.state = DockStage.ON_SHORE

        # Transform variables
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Policy variables 
        self._default_policy = StopPolicy()
        self._policy_queue: deque[Policy] = deque()
        self._current_state = PolicyState(
            time=self.get_clock().now(),
            position=np.zeros(2),
            velocity=np.zeros(2),
            acceleration=np.zeros(2),
            target=np.zeros(2)
        )

        # Not used
        self.target_center = np.zeros(2)
        self.points = np.zeros((2, 1))
        self.measurements_x = []
        self.measurements_y = []  

        # Test 

        
        ###
        ### SUBSCRIBERS
        ###

        # Test subscribers (Not needed in real life)
        self.imu_sub = self.create_subscription(Imu, '/usv/imu/data', self._imu_callback, 10)
        # Lidar subscriber
        self.laser_sub = self.create_subscription(PointCloud2, '/usv/slot6/points', self._laser_cb, 10)
        
        # Coordinator subscriber
        self.USV_to_shore_sub = self.create_subscription(Bool, 'USV_to_shore', self._USV_to_shore_callback, 10)
        self.POI_initial_coords_sub = self.create_subscription(Point, 'POI_initial_coordinates', self._POI_initial_coords_callback, 10)
        
        # Camera detection subscriber
        self.POI_camera_coords_sub = self.create_subscription(Point, 'POI_camera_coordinates', self._POI_camera_coords_callback, 10)

        # USV angle subscriber
        self.yaw_USV_sub = self.create_subscription(Float32, 'yaw_USV', self._yaw_USV_callback, 10)

        ###
        ### PUBLISHERS
        ###

        # Publish velocity commands
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publish turn angle and velocity to control the USV (x: angle, y: velocity)
        self.usv_control_pub = self.create_publisher(Vector3, 'usv_control', 10)

        # RVIZ publishers
        self.center_pub = self.create_publisher(Marker, 'target_center', 10)
        self.target_pub = self.create_publisher(Marker, 'target', 10)
        self.traj_pub = self.create_publisher(Marker, 'trajectory', 10)
        self.line_pub = self.create_publisher(Marker, 'line', 10)
        self.scan_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.pc_crop_pub = self.create_publisher(PointCloud2, 'pc_crop', 10)

        # Coordinator publisher with USV state
        self.usv_state_pub = self.create_publisher(Int16, 'USV_state', 10)
        

        ###
        ### CREATE TIMERS
        ###

        self.control_timer = self.create_timer(self.DT, self._control_cb)
        self.USV_state_publisher_timer = self.create_timer(0.2, self._usv_state_publisher)

        # Create action server
        """ self._action_server = ActionServer(
            self, Dock, 'dock',
            self._execute_callback) """
        

    ### 
    ### CALLBACKS
    ###

    def _yaw_USV_callback(self, yaw_msg: Float32) -> None: 

        self.yaw_USV = yaw_msg.data
        self.yaw_USV_received = True
        
    def _POI_initial_coords_callback(self, POI_initial_coords_msg: Point) -> None:
        # Read POI coordinates
        self.POI_initial_coords = [[POI_initial_coords_msg.x], [POI_initial_coords_msg.y]]
        self.POI_initial_received = True

        # TO DO: Calculate POI angle
        self.POI_initial_angle = 60.0  # deg
        self.get_logger().info(f"POI coordinates received: {POI_initial_coords_msg}")

    def _POI_camera_coords_callback(self, POI_camera_coords_msg: Point) -> None:
        # Read POI coordinates
        self.POI_camera_coords = [[POI_camera_coords_msg.x], [POI_camera_coords_msg.y]]
        self.POI_camera_received = True

        # TO DO: Calculate POI angle
        self.POI_camera_angle = 0.0  # deg
        self.get_logger().info(f"Target camera coordinates received: {self.POI_camera_coords}")
        self.get_logger().info(f"Target camera angle calculated: {self.POI_camera_angle}")


    def _USV_to_shore_callback(self, USV_to_shore: Bool) -> None: 
        # TO DO: Read flag to come back to shore after picking up the boxes
        self.get_logger().info("Received message from the coordinator to come back to shore")

        self.get_logger().info(f"Message USV_to_shore: {USV_to_shore}")

    # TEST CALLBACKS
    def _imu_callback(self, imu_msg) -> None: 
        orientation_list = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]
        _, _, self.yaw_USV = np.rad2deg(euler_from_quaternion(orientation_list))

        #self.get_logger().info(f"Yaw received: {self.yaw_USV}")


    ###
    ### STATE MACHINE
    ###

    def _usv_state_publisher(self): 

        # Initial state is ON_SHORE until we receive the POI_initial_coordinates msg

        ### ON_SHORE ###############################################################

        if self.state == DockStage.ON_SHORE and self.POI_initial_received:
            self.state = DockStage.LEAVE_GATE


        ### LEAVE_GATE ###############################################################

        if self.state == DockStage.LEAVE_GATE:
            # Go straight until we leave the gate (angle = 0.0 / velocity = self.leave_gate_vel)

            angle = 0.0     # deg     
            velocity = self.leave_gate_vel  # m/s
            
            usv_control_msg = Vector3()
            usv_control_msg.x = angle
            usv_control_msg.y = velocity
            self.usv_control_pub.publish(usv_control_msg)

            # TO DO: Condition to finish the state
            # Check with lidar distance to the closest points that are not the boat, so 
            # we know that when they are farther than the distance to from the lidar to
            # the USV end point we know we are in the sea

            time.sleep(2)
            self.state = DockStage.TURNING_TO_POI
            # Set the initial turning yaw for the next state
            self.initial_turn_yaw = self.yaw_USV
            self.get_logger().info(f"Initial turn yaw = {self.initial_turn_yaw}")


        ### TURNING_TO_POI ###############################################################
        
        if self.state == DockStage.TURNING_TO_POI: 
            # Turn (angle = POI direction / velocity = 0.0)

            # Send the POI angle once to the USV controller to turn until it reaches it
            if self.USV_control_msg_sent == False: 
                angle = self.POI_initial_angle     # deg   
                velocity = 0.0  # m/s
                
                self.pulish_USV_control_msg(angle, velocity)

                # Set flag to True to not send the command again during this state
                self.USV_control_msg_sent = True

            # Calculate current turned angle
            
            self.get_logger().info(f"Initial turn yaw = {self.initial_turn_yaw} / Current yaw = {self.yaw_USV}")
            current_turn = abs(self.initial_turn_yaw - self.yaw_USV)
            self.get_logger().info(f"Turning to POI: {current_turn}")
            

            ## TO DO : Finish the state when target angle reached (Could be done with the following statement)
            # Keep turning until current turn (difference between curr ang and the init ang) close to POI_initial_angle
            if current_turn < (self.POI_initial_angle + self.turning_threshold) and current_turn > (self.POI_initial_angle - self.turning_threshold):

                # Once the POI angle is reached, change state
                self.state = DockStage.STRAIGHT_TO_POI_BLIND

                # Set flag to True to send the following command for the next state
                self.USV_control_msg_sent = False


        ### STRAIGHT_TO_POI_BLIND ###############################################################
                        
        if self.state == DockStage.STRAIGHT_TO_POI_BLIND: 
            # Go straight until POI detected by camera (angle = 0.0 / velocity = )

            # Send the straight command once to the USV controller to gi straight until the camera detects the POI
            # Quit the if statement if message needs to be sent every loop 
            if self.USV_control_msg_sent == False: 
                
                self.pulish_USV_control_msg(angle = 0.0, velocity = self.straight_to_poi_blind_vel)

                # Set flag to True to not send the command again during this state
                self.USV_control_msg_sent = True
            
            # Once camera detects the POI, change the state
            if self.POI_camera_received: 
                self.state = DockStage.STRAIGHT_TO_POI_CAMERA

                # Set flag to True to send the following command for the next state
                self.USV_control_msg_sent = False


        ### STRAIGHT_TO_POI_CAMERA ###############################################################
                
        if self.state == DockStage.STRAIGHT_TO_POI_CAMERA and self.POI_camera_received:

            self.pulish_USV_control_msg(angle = self.POI_camera_angle, velocity = self.straight_to_poi_camera_vel) 

            if self.POI_lidar_detected == True: 
                self.state = DockStage.STRAIGHT_TO_POI_LIDAR
                self.POI_camera_received = False

        # TO DO: Maybe, if we are in STRAIGHT_TO_POI_CAMERA and we do not receive POI_camera _received, come back to
        # POI_STRAIGHT_BLIND or another state to look for the target moving the camera. And then when detected again 
        # change to STRAIGHT TO POI CAMERA

        
        ### STRAIGHT_TO_POI_LIDAR ###############################################################
        
        if self.state == DockStage.STRAIGHT_TO_POI_LIDAR and self.POI_lidar_detected and self.lidar_msg_received: 

            # TO DO: Calculate POI angle from target cluster

            self.pulish_USV_control_msg(angle = self.POI_camera_angle, velocity = self.straight_to_poi_camera_vel) 
            self.lidar_msg_received = False


        self.get_logger().info(f"CURRENT STATE: {self.state}")




    def _control_cb(self):
        print("CONTROL CALLBACK")
        """Control loop, called every 0.1 seconds. Executes current policy action."""
        try:

            # RVIZ clear path publisher
            if self.colision_status == True: 
                line_color = [1.0, 0.0, 0.0]
            else: 
                line_color = [0.0,1.0, 0.0]

            """ self.publish_line(start=[self.clear_path_width/2, 0.0], end=[self.clear_path_width/2, self.clear_path_length], id = 0, color = line_color)
            self.publish_line(start=[-self.clear_path_width/2, 0.0], end=[-self.clear_path_width/2, self.clear_path_length], id = 1, color = line_color)
            """
            self._current_state.target = self.target_center

            # Track line using Kalman Filter
            #if self.target_detected:
                #self._track_line()


            # JUST FOR DEBUG PURPOSES
            # Save measurements, predictions and updates to analyze performance
            if self.debug == True:
                #print(self.step_counter)
                if self.step_counter > 11000: 

                    with open('updated_points.json', 'w') as jsonfile:
                        json.dump(self.updated_points, jsonfile)
                    with open('detected_points.json', 'w') as jsonfile:
                        json.dump(self.detected_points, jsonfile)
                    with open('predicted_points.json', 'w') as jsonfile:
                        json.dump(self.predicted_points, jsonfile)
            #
                
            ##print(self._policy_queue)
            action = self._get_action(self._current_state)

            #print(f"USV State: {self.state}")

            # If no collision detected publish action velocities calculated
            if self.colision_status == False:
                msg = Twist()
                msg.linear.x = action.linear_velocity[0]
                msg.linear.y = action.linear_velocity[1]
                msg.linear.z = action.linear_velocity[2]
                msg.angular.x = action.angular_velocity[0]
                msg.angular.y = action.angular_velocity[1]
                msg.angular.z = action.angular_velocity[2]
                self.twist_pub.publish(msg)
            else: 
                # If there is an obstacle in the path, stop the USV until the obstacle goes out the path
                msg = Twist()
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z =  0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0

        except Exception as e:
            self.get_logger().error(f'Uncaught exception: {traceback.format_exc()}')

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> Dock.Result:
        """Callback for the action server."""
        try:
            self.get_logger().info('Executing goal...')

            policy = TargetPolicy(self.v_orbit, self.r_orbit, 2.0, self.get_logger())
            self._policy_queue.append(policy)

            orbit_policy = OrbitPolicy(self.r_orbit, self.v_orbit, self.get_logger())
            self._policy_queue.append(orbit_policy)

            self.state = DockStage.APPROACH
            while not self.state == DockStage.STOP:
                time.sleep(self.DT)

            goal_handle.succeed()
            result = Dock.Result(success=True)
            return result
        except Exception as e:
            self.get_logger().error(f'Uncaught exception: {traceback.format_exc()}')
            goal_handle.abort()
            return Dock.Result(success=False)

    def _laser_cb(self, pointcloud: PointCloud2) -> None:
        """Callback for the laser scan topic."""

        # Initialize target_detected = False 
        #self.target_detected = False
        self.POI_lidar_detected = False

        # Get the center_line from the PointCloud2 and delete vessel points
        center_line = self._pc2_to_scan(pointcloud=pointcloud, ang_threshold=[1, -1])

        # Prepare points to process them
        self.points3d = np.array(center_line).T # 3xN points

        if self.points3d.shape[0] != 0:
            #self.get_logger().info(f"Points3d: {self.points3d.shape}")
            self.points = self.points3d[:2, :]      

            ###
            ### POINCLOUD CLUSTERING
            ###
            
            if self.POI_initial_received == True:
                # Cluster the whole pointcloud looking for the POI and detecting obstacles. Manage obstacle avoidance
                self.pointcloud_clustering()

                if self.POI_lidar_detected == True: 
                    # If the target has been detected, look for the side lines
                    self.target_line_clustering()
        
        # Set flag to True when receiveing a pointcloud
        self.lidar_msg_received = True


    def pulish_USV_control_msg(self, angle, velocity): 
        usv_control_msg = Vector3()
        usv_control_msg.x = float(angle)
        usv_control_msg.y = float(velocity)
        self.usv_control_pub.publish(usv_control_msg)


    def target_line_clustering(self):
        ''' Cluster the lines detected in the target vessel'''

        points = self.tracked_target_points[:2, :]

        # Get the min and max points
        min_x, min_y = np.min(points, axis=1)
        max_x, max_y = np.max(points, axis=1)

        # Apply hough lines to an image created with the scan
        self.find_lines(points)

        # Filter lines detected from hough lines
        filtered_lines = self.detected_lines_filter(points, min_x, min_y)

        vessel_sides = self.get_vessel_sides(filtered_lines)

        

    def get_vessel_sides(self, filtered_lines):

        # If there are several lines, select best line as the largest with more points
        if len(filtered_lines)>0:
            discarded_lines = []
            best_line = filtered_lines[0]
            best_n_points = filtered_lines[0].n_points()
            for idx, line in enumerate(filtered_lines): 
                line_length = line.length()
                if line_length > 2: 
                    if line.n_points() > best_n_points:
                        best_line = line
                    else:
                        discarded_lines.append(line)
                else: 
                    discarded_lines.append(line)

            # compare best line with  the others to find 90 degrees (back side)
            """ start_point1 = best_line.start
            end_point1 = best_line.end
            for line in discarded_lines: 
                start_point2 = line.start
                end_point2 = line.end
                angle = angle = self.angle_between_lines(start_point1, end_point1, start_point2, end_point2)

                if angle < 95 and angle > 85: 
                    best_line_2 = line
                    self._visualize_line(list(line.start), list(line.end), id=0) """

            line_msg = self._visualize_line(list(best_line.start), list(best_line.end), id=1) 

            # Get points belonging to perpendicular detected side to detect largest and shortest vessel sides
            valid_points_array = self.get_second_vessel_side(best_line, self.tracked_target_points[:2, :])

        else: 
            self.get_logger().info("LINE NOT DETECTED")
    
    def get_second_vessel_side(self, best_line, points):
        ''' Get perpendicular line to a line containing a point'''

        # Try to find second line through the best line start point
        best_line_2 = self.find_second_vessel_side(best_line, best_line.start, points)

        # If second side not found, try with best line end point
        if best_line_2 == None: 
            best_line_2 = self.find_second_vessel_side(best_line, best_line.end, points)

        # Publish line to rviz if founded
        if best_line_2 != None:
            self._visualize_line(list(best_line_2.start), list(best_line_2.end), id=15)
            """ lines_array_msg = MarkerArray()
            lines_array_msg.markers = []
            for i in range(len(valid_points_array[0])): 
                x = valid_points_array[0][i]
                y = valid_points_array[1][i]
                marker = self.get_marker([x, y], [0.0, 1.0, 0.0], scale=0.1, id_=int(i))
                lines_array_msg.markers.append(marker)
            self.scan_pub.publish(lines_array_msg) """
    
    def find_second_vessel_side(self, best_line, point, points): 
        ''' Try to find the second side of the vessel given the first found side
            and one of the extreme points of the side found'''
        # Initialize best_line_2
        best_line_2 = None

        # Caculate perpendicular line containing the start  of the best line        
        side_lin_eq = self.get_perpendicular_line(best_line.line_eq(), point) 
        
        _, inliers, _ = all_points_valid(line=np.array(side_lin_eq), points=points, threshold=0.2)

        # Get inhomogeneous coordinates
        valid_points_array = inliers[:2,:]

        # Discard points belonging to best_line
        valid_points_filter = ~np.isin(valid_points_array, best_line.points())
        valid_points_array = valid_points_array[:,valid_points_filter[0]]
        valid_points_array = np.hstack((valid_points_array, point.reshape(-1, 1)))

        if valid_points_array.shape[1] != 0: 
                # Fit line to valid_points_array
                valid_points_eq = est_line(valid_points_array)

                # Get first and last line points
                first_point, last_point = self.get_first_last_line_points(valid_points_array, valid_points_eq)
                
                # Calculate angle between lines
                angle = angle = self.angle_between_lines(best_line.start, best_line.end, first_point, last_point)

                if angle > 85 and angle < 95: 
                    best_line_2 = Line(eq=valid_points_eq, start = first_point, end = last_point, pts=valid_points_array)

        return best_line_2
    
    def get_first_last_line_points(self, valid_points_array, valid_points_eq):
        # Compute the orthogonal vector [-B, A]
        orthogonal_vector = np.array([-valid_points_eq[1], valid_points_eq[0]])

        # Project all points onto the orthogonal vector and sort them
        projections = np.dot(valid_points_array.T, orthogonal_vector)
        sorted_indices = np.argsort(projections)

        # The first point is the one with the smallest projection value, and the last point is the one with the largest projection value
        first_point = valid_points_array[:, sorted_indices[0]]
        last_point = valid_points_array[:, sorted_indices[-1]] 

        return first_point, last_point
    
    def get_perpendicular_line(self, line_eq, point): 
        ''' Get perpendicular line given a line eq and a point'''

        # Get line params
        a, b, c = line_eq
        
        # Get point
        x, y = point
        
        # Calculate the slope of the original line
        slope_original = -a / b

        # Calculate the slope of the perpendicular line
        slope_perpendicular = -1 / slope_original

        # Calculate the intercept of the perpendicular line
        c_perpendicular = y - slope_perpendicular * x

        # Coefficients of the perpendicular line equation
        a_perpendicular = slope_perpendicular
        b_perpendicular = -1
        c_perpendicular = c_perpendicular

        return [a_perpendicular, b_perpendicular, c_perpendicular]

    def angle_between_lines(self, start_point1, end_point1, start_point2, end_point2):
        vector1 = np.array([end_point1[0] - start_point1[0], end_point1[1] - start_point1[1]])
        vector2 = np.array([end_point2[0] - start_point2[0], end_point2[1] - start_point2[1]])

        dot_product = np.dot(vector1, vector2)
        magnitude_v1 = np.linalg.norm(vector1)
        magnitude_v2 = np.linalg.norm(vector2)

        cosine_theta = dot_product / (magnitude_v1 * magnitude_v2)

        # Ensure the value is within the valid range for arccosine
        cosine_theta = np.clip(cosine_theta, -1.0, 1.0)

        theta = np.arccos(cosine_theta)
        angle_degrees = np.degrees(theta)
        return angle_degrees

    def pointcloud_clustering(self): 
        ###
        ### DBSCAN CLUSTERING
        ###

        labels, label_idxs = self.dbscan_clustering()

        # Iterate over all clusters and find the target
        """ lines_array_msg = MarkerArray()
        lines_array_msg.markers = [] """
        updated_tracked_object = np.zeros(len(self.trackers_objects))

        # Itarate over DBSCAN mask to get target and obstacle clusters
        self.detect_target_and_obstacles(labels, label_idxs, updated_tracked_object)

        # Redetect target if olcuded by an obstacle moving in front of the USV
        self.redetect_target_if_ocluded(labels, label_idxs)

        # Remove tracked objects that have not been detected
        self.remove_not_detected_objects(updated_tracked_object)
        
        # Check obstacle collision in the path in front of the vessel
        self.check_obstacle_collision()

        # If collision_status = True -> Collision detected / False -> Free path, no collisions
        self.colision_status = np.any(np.array(self.collision_objects))



        #print(f"Collision status: {self.colision_status}")
        """self.get_logger().info("///////////////////////////////////////////////////////")
        self.get_logger().info(f"Target tracked: {self.POI_lidar_detected}")
        if self.POI_lidar_detected == True: 
            self.get_logger().info(f"Target pose: {self.tracker_target.pos}")

        for k, object_tracked in enumerate(self.trackers_objects): 
            self.get_logger().info(f" Object {k} tracked: {object_tracked.pos}")
            try: 
                angle = np.rad2deg(math.asin(object_tracked.pos[0]/object_tracked.pos[1]))
            except:
                test = self.get_logger().info(f"Asin: {object_tracked.pos[0]/object_tracked.pos[1]}")
                self.get_logger().info(f" Error: {test}")
                time.sleep(20)
                    
            self.get_logger().info(f" Object {k} tracked angle: {angle}") """
        self.get_logger().info(f"Colision status: {self.colision_status}")

    def check_obstacle_collision(self): 
        ###
        ### CHECK OBSTACLE COLLISION
        ###

        for k, object_tracked in enumerate(self.trackers_objects): 
            
            # Check if the cluster detected is inside the area defined (rectangle in front of the USV)
            if np.any(self.tracked_objects_points[k][1,:] < self.clear_path_length ) and np.any(self.tracked_objects_points[k][1,:] > 0) \
               and np.any(self.tracked_objects_points[k][0,:] > -self.clear_path_width/2) and np.any(self.tracked_objects_points[k][0,:] < self.clear_path_width/2):
                color = [255.0, 0.0, 0.0]
                # Set collision object to 1 if is inside the trajectory
                self.collision_objects[k] = 1
            else: 
                color = [0.0, 255.0, 0.0]
                # Set collision object to 1 if it is outside the path
                self.collision_objects[k] = 0
            # Publish a point tracking the obstacle
            self.center_pub.publish(self.get_marker(object_tracked.pos, color = color, scale = 0.3, id_ = k+100))
            #print(f" Object {k} tracked: {object_tracked.pos}")s

    def remove_not_detected_objects(self, updated_tracked_object): 
        ### REMOVE TRACKED OBJECTS THAT HAVE NOT BEEN DETECTED

        eliminate_tracked_objects_idxs = list(np.where(updated_tracked_object == 0)[0])
        eliminate_tracked_objects_idxs.sort(reverse=True)
        
        # Remove elements at specified indices using a loop
        if len(eliminate_tracked_objects_idxs) > 0:
            for index in eliminate_tracked_objects_idxs:
                self.trackers_objects.pop(index)
                self.tracked_objects_points.pop(index)
                self.collision_objects.pop(index)

    def redetect_target_if_ocluded(self, labels, label_idxs): 
        ### IF TARGET HAS BEEN OCLUDED, TRY TO FIND IT AGAIN

        if self.POI_lidar_detected == False and self.tracker_target != None: 
            # If tracker target is not None, because the target has been detected previously but it is not being detected currently, 
            # check if some of the objects being detected currently could be the target by comparing the minimum distance of the
            # cluster points with the target tracked. 
            self.tracker_target.predict()

            for label_idx in label_idxs:
                # Get points from each cluster detected
                cluster_2dpoints  = self.points3d[:,np.where(labels == label_idx)[0]][0:2,:]

                points2target_distances = np.array([np.linalg.norm(self.tracker_target.pos - point) for point in cluster_2dpoints.T])

                cluster_points  = self.points3d[:,np.where(labels == label_idx)[0]]
                
                if np.any(points2target_distances < self.ocluded_target_threshold) == True:
                    # Get median from the cluster
                    cluster_2d_pos = np.median(cluster_2dpoints, axis=1)[:,None]
                    
                    # Update target tracker with the measurement
                    self.tracker_target.update(cluster_2d_pos.reshape(2,1))
                    
                    # Save target points
                    self.tracked_target_points = cluster_points

                    # Target detected
                    self.POI_lidar_detected = True
                    
            self.center_pub.publish(self.get_marker(self.tracker_target.pos.T, color = [0.0, 0.0, 255.0], scale = 0.3, id_ = 99999))


    def detect_target_and_obstacles(self, labels, label_idxs, updated_tracked_object): 
        ''' Function that iterates over the mask created by DBSCAN and search 
            for the target as well as the possible obstacles'''
        
        # Iterate over the DBSCAN mask
        for label_idx in label_idxs:
            # Get points from each cluster detected
            cluster_points  = self.points3d[:,np.where(labels == label_idx)[0]]

            # Get median from each cluster
            cluster_median_pos = np.median(cluster_points, axis=1)
            cluster_2d_pos = cluster_median_pos[0:2, None]
            
            # Check if the cluster has a minimum number of points
            if cluster_points.shape[1] > self.lidar_cluster_min_points:
                
                # If POI camera received
                if self.POI_camera_received: 

                    # If tracker_target not initialized, try to find the target matching camera and lidar target angle detection
                    if self.tracker_target == None:

                        # Calculate cluster angle
                        try: 
                            cluster_angle = self.get_object_orientation(cluster_2d_pos[0], cluster_2d_pos[1])
                        except: 
                            test_angle = cluster_2d_pos[0]/cluster_2d_pos[1]
                            self.get_logger().info(f"coords2: {cluster_2d_pos[0]}, {cluster_2d_pos[1]}")
                            self.get_logger().info(f"Angle: {test_angle}")
                            time.sleep(20)

                        # Compare camera target detected angle with cluster detected angle
                        POI_angle_difference = self.POI_camera_angle - cluster_angle
                        self.get_logger().info(f"POI_angle_diif: {POI_angle_difference}")
                        # Initialize target tracker if angle threshold 
                        if POI_angle_difference < self.match_POI_cam_lid_angle or POI_angle_difference > -self.match_POI_cam_lid_angle: 
                            # Initialize target tracker
                            self.tracker_target = Tracker2D(cluster_2d_pos[:,0], self.P0, self.R0, self.Q0, self.DT, self.mode)
                            self.tracked_target_points = cluster_points
                            self.center_pub.publish(self.get_marker(self.tracker_target.pos, color = [0.0, 0.0, 255.0], scale = 0.3, id_ = 99999))
                            self.POI_lidar_detected = True
                        # Update cluster as an object if it is not the target
                        else: 
                            # Update cluster as an object
                            updated_tracked_object = self.update_objects_measurements(cluster_2d_pos, cluster_points, updated_tracked_object)
                    
                    # Once we have the target, compare distances to update target every time
                    else: 

                        # Calculate cluster angle
                        try: 
                            cluster_angle = self.get_object_orientation(cluster_2d_pos[0], cluster_2d_pos[1])
                        except: 
                            test_angle = cluster_2d_pos[0]/cluster_2d_pos[1]
                            self.get_logger().info(f"coords1: {cluster_2d_pos[0]}, {cluster_2d_pos[1]}")
                            self.get_logger().info(f"Angle: {test_angle}")
                            time.sleep(20)
                        # Calculate distance with target tracker if initialized
                        cluster_target_distance = np.linalg.norm(self.tracker_target.pos - cluster_2d_pos[:,0])
                
                        ### IF THE CLUSTER IS THE TARGET, UPDATE MEASUREMENT

                        # If target match update position
                        if cluster_target_distance < self.distance_threshold: 
                            self.get_logger().info(f"TARGET Angle: {cluster_angle}")
                            # Update target tracker
                            self.tracker_target.predict()
                            self.tracker_target.update(cluster_2d_pos)
                            self.tracked_target_points = cluster_points
                            
                            self.center_pub.publish(self.get_marker(self.tracker_target.pos.T, color = [0.0, 0.0, 255.0], scale = 0.3, id_ = 99999))
                            self.POI_lidar_detected = True
                        # If cluster is not the target, update objects 
                        else: 
                            # Update cluster as an object
                            updated_tracked_object = self.update_objects_measurements(cluster_2d_pos, cluster_points, updated_tracked_object)

                else: 
                    # Update cluster as an object
                    updated_tracked_object = self.update_objects_measurements(cluster_2d_pos, cluster_points, updated_tracked_object)

                # Fill marker msg for cluster visualization in RVIZ
                """ for i in range(cluster_points.shape[1]): 
                    x = cluster_points[0, i]
                    y = cluster_points[1, i]
                    marker = self.get_marker([x, y], self.cluster_colors[int(label_idx)], scale=0.1, id_=int(i))
                    lines_array_msg.markers.append(marker)
            self.scan_pub.publish(lines_array_msg) """
                
        return updated_tracked_object
    
    def get_object_orientation(self, x,y):  #left hand rule for simulated data
        if x>0 and y>0:
                angle = self.atan(x,y)
        if y<0:
                angle = 180+self.atan(x,y)
        if x<0 and y>0: 
                angle = 360-self.atan(x,y) 
        return angle
    
    def atan(self, x, y): 
        return np.rad2deg(math.atan(x/y))
                
    def update_objects_measurements(self, cluster_2d_pos, cluster_points, updated_tracked_object): 
        object_tracked = False
        if len(self.trackers_objects) > 0: 
            
            for object_tracker_idx in range(len(self.trackers_objects)): 
                cluster_object_distance = np.linalg.norm(self.trackers_objects[object_tracker_idx].pos - cluster_2d_pos[:,0])

                # Check if the object is already being tracked and update with measurement
                if cluster_object_distance < 1: 
                    self.trackers_objects[object_tracker_idx].predict()
                    self.trackers_objects[object_tracker_idx].update(cluster_2d_pos)
                    self.tracked_objects_points[object_tracker_idx] = cluster_points
                    object_tracked = True
                    updated_tracked_object[object_tracker_idx] = 1
                    break
            
            # If the object is not being tracked, initialize tracker
            if object_tracked == False: 
                self.trackers_objects.append(Tracker2D(
                    cluster_2d_pos[:,0], self.P0, self.R0, self.Q0, self.DT, self.mode))
                self.tracked_objects_points.append(cluster_points)
                # Initialize tracked obstacle without collision to check it later
                self.collision_objects.append(0)
                updated_tracked_object = np.append(updated_tracked_object, 1)
        
        # start tracking the object if the tracker list has not been initialized
        else: 
            self.trackers_objects.append(Tracker2D(
                    cluster_2d_pos[:,0], self.P0, self.R0, self.Q0, self.DT, self.mode))
            self.tracked_objects_points.append(cluster_points)
            # Initialize tracked obstacle without collision to check it later
            self.collision_objects.append(0)
            updated_tracked_object = np.append(updated_tracked_object, 1)

        return updated_tracked_object

    def dbscan_clustering(self): 
        # Standardize the data
        scaler = StandardScaler()
        point_cloud_scaled = scaler.fit_transform(self.points3d.T)

        # Apply DBSCAN
        dbscan = DBSCAN(eps=self.epsilon, min_samples=self.min_samples)
        labels = dbscan.fit_predict(point_cloud_scaled)

        # Get cluster labels
        label_idxs = np.unique(labels)

        return labels, label_idxs

    def _pc2_to_scan(self, pointcloud: PointCloud2, ang_threshold = [-0.5, 5]): 
        """Get an horizontal scan from a PointCloud2"""
        scan_line = []
        
        for p in point_cloud2.read_points(pointcloud, field_names = ("x", "y", "z"), skip_nans=True):
            # Get XYZ coordinates to calculate vertical angle and filter by vertical scans
            x = p[0]
            y = p[1]
            z = p[2]
            d = math.sqrt(x**2 + y**2 + z**2)
            if d != 0: 
                # Calculate vertical angle
                angle = 90 - np.rad2deg(np.arccos(z/d))

                # Save points from center scan
                if (angle < ang_threshold[0] and angle > ang_threshold[1]) and ((x > 0.1) or (x < -3) or (x > -3 and x < 0 and (y > 0 or y < -4))): 
                    scan_line.append([x, y, z])
                    
        pc2_cropped = PointCloud2()
        pc2_cropped.header = pointcloud.header
        pc2_cropped.height = 1
        pc2_cropped.width = len(scan_line)
        pc2_cropped.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
        pc2_cropped.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
        pc2_cropped.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
        pc2_cropped.is_bigendian = False
        pc2_cropped.point_step = 12  # 4 (x) + 4 (y) + 4 (z) bytes per point
        pc2_cropped.row_step = pc2_cropped.point_step * len(scan_line)
        pc2_cropped.data = np.array(scan_line, dtype=np.float32).tobytes()
        self.pc_crop_pub.publish(pc2_cropped) 

        return np.array(scan_line)


    def _track_line(self) -> None:
        """Detects and tracks a line in the laser scan."""
        """ if self.state == DockStage.STOP:
            return """

        lines = detect_lines(self.tracked_target_points[0:2, :], 4, 0.1, self.get_logger())
        
        if self.best_line == None: 
            return

        if len(lines) == 0:
            return

        self.best_line = lines[0]
        for line in lines:
            if line.length() > self.best_line.length():
                self.best_line = line

        """ if self.best_line.length() < self.line_thres_min and self.best_line.length() > self.line_thres_max:
            self.get_logger().debug(f'Line not found')
            return """

        """ if self.tracked_start is not None:
            self._visualize_line() """

        #lower, higher = self._order_line_endpoints(self.best_line.start, self.best_line.end)
        """ lower = self.best_line.start
        higher = self.best_line.end
        h = np.linalg.norm(higher)
        l = np.linalg.norm(lower)
        r = np.linalg.norm(higher - lower) """
        #angle = np.arccos((r ** 2 + l ** 2 - h ** 2) / (2 * r * l))
        #self.get_logger().info(f'Line angle: {self.prev_angle:.1f}°')

        """ if angle < self.angle_threshold and self.tracked_start is None:
            self.get_logger().warn(f'Too small line angle: {np.rad2deg(angle):.1f}° < {np.rad2deg(self.angle_threshold):.1f}°, ignoring...')
            return """

        if self.tracked_start is not None:
            #self.tracked_start.update(lower[:, None])
            self.tracked_start.predict()
            #self.tracked_end.update(higher[:, None])
            self.tracked_end.predict()

            if self.debug == True:
                self.step_counter += 1
                self.predicted_points.append([self.step_counter, self.tracked_end.pos[0], self.tracked_end.pos[1]])
            
        else:
            self.get_logger().info(f'Started tracking line!')
            """ self.state = DockStage.APPROACH """

            self.tracked_start = Tracker2D(
                self.best_line.start, self.P0, self.R0, self.Q0, self.DT, self.mode)
            self.tracked_end = Tracker2D(
                self.best_line.end, self.P0, self.R0, self.Q0, self.DT, self.mode)

            """ self._policy_queue.clear()
            for target, speed in zip(self.trajectory_targets, self.trajectory_speeds):
                policy = ApproachPolicy(
                    target, speed, 1.0, self.get_logger()
                )
                self._policy_queue.append(policy)

            touch_policy = TouchPolicy(
                self.tracked_start, self.tracked_end, self.usv_width,
                self.v_touch, self.touch_precision,
                self._set_stop,
                self.get_logger()
            )
            self._policy_queue.append(touch_policy)
            self.get_logger().info(f'Initial x0: {self.tracked_start.x}') """
        self._recalculate_approach_trajectory(self.tracked_end.pos, self.tracked_start.pos)

    def _set_stop(self) -> None:
        """Callback for the touch policy."""
        self.state = DockStage.STOP
        self.tracked_start = None
        self.tracked_end = None
        self._policy_queue.clear()

    def _recalculate_approach_trajectory(self, lower: np.ndarray, higher: np.ndarray) -> None:
        """Recalculates the approach trajectory based on the detected line."""
        xa = (higher - lower) / np.linalg.norm(higher - lower)
        R = np.eye(2)
        R[0:2, 0] = xa
        R[0, 1] = -R[1, 0]
        R[1, 1] = R[0, 0]

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.BASE_LINK
        msg.scale.x = msg.scale.y = msg.scale.z = 0.3
        msg.type = Marker.SPHERE_LIST
        msg.points = []
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 0.5
        msg.color.a = 1.0

        for i in range(len(self.trajectory_relative)):
            p = self.trajectory_relative[i]
            self.trajectory_targets[i][:] = lower[:, None] + R @ p

            msg.points.append(
                Point(x=self.trajectory_targets[i][0, 0], y=self.trajectory_targets[i][1, 0], z=0.0)
            )

        self.traj_pub.publish(msg)

    @staticmethod
    def _order_line_endpoints(first: np.ndarray, second: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Orders the endpoints of a line so that the second is to the left of the line
        defined by the origin and the first point."""
        if np.cross(first, second) > 0:
            return first, second
        else:
            return second, first

    def _get_action(self, state: PolicyState) -> PolicyAction:
        """Returns the action for the first policy in the queue."""
        success = False

        while not success:
            # use default policy if nothing else is in the queue
            if len(self._policy_queue) == 0:
                self._policy_queue.append(self._default_policy)
            elif len(self._policy_queue) >= 2 and self._policy_queue[0] is self._default_policy:
                self._policy_queue.popleft()

            policy = self._policy_queue[0]
            ##print(policy)
            try:
                action = policy.get_action(state)
                if isinstance(policy, TargetPolicy):
                    end = policy.end
                    self.get_logger().info(f'end point: {end}')
                    marker = self.get_marker(end)
                    self.target_pub.publish(marker)
                success = True
                break
            except (PolicyCanceledException, PolicyOutOfBoundsException) as e:
                self.get_logger().info(f'Error {e}\nRemoving from queue')
                policy.cancel()
                if len(self._policy_queue) > 0:
                    self._policy_queue.popleft()

        return action

    def _scan_to_points(self, scan: LaserScan) -> np.ndarray:
        """Converts a laser scan to a 2D array of points in the base_link frame."""
        n_points = len(scan.ranges)
        T = self._get_transform(self.BASE_LINK, scan.header.frame_id)

        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, n_points)
        angles = angles[np.isfinite(ranges)]
        ranges = ranges[np.isfinite(ranges)]
        n_good_points = len(ranges)

        points_lidar = np.array([
            ranges * np.cos(angles),
            ranges * np.sin(angles),
            np.zeros(n_good_points),
            np.ones(n_good_points)
        ])
        points_base = T @ points_lidar
        return points_base[:2, :]   # drop z

    def get_marker(self, point: np.ndarray, color = None, stamp=None, id_=0, scale = 0.3) -> Marker:
        """Returns a marker for the given point."""
        if color is None:
            color = (1.0, 0.0, 0.0)

        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        msg = Marker()
        msg.header.stamp = stamp
        msg.header.frame_id = 'usv/sensor_6/sensor_link/lidar'
        msg.type = Marker.SPHERE
        msg.id = id_
        msg.pose.position.x = point[0]
        msg.pose.position.y = point[1]
        msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = scale
        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 1.0

        return msg

    def _visualize_line(self, first_point, last_point, id) -> None:
        """Visualizes the tracked line."""
        msg = Marker()
        msg.id = id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'usv/sensor_6/sensor_link/lidar'
        msg.scale.x = 0.1
        msg.type = Marker.LINE_LIST
        msg.points = []
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 1.0

        start_ = Point(x=first_point[0], y=first_point[1], z=0.0)
        end_ = Point(x=last_point[0], y=last_point[1], z=0.0)

        """ start_ = Point(x=self.best_line.start[0], y=self.best_line.start[1], z=0.0)
        end_ = Point(x=self.best_line.end[0], y=self.best_line.end[1], z=0.0) """

        msg.points.append(start_)
        msg.points.append(end_)

        self.line_pub.publish(msg)

    def publish_line(self, start, end, id, color = [255.0, 0.0, 0.0]): 
        """Visualizes line."""
        msg = Marker()
        msg.id = id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'usv/sensor_6/sensor_link/lidar'
    
        msg.scale.x = 0.1
        msg.type = Marker.LINE_LIST
        msg.points = []
        msg.color.r = color[0]
        msg.color.g = color[1]
        msg.color.b = color[2]
        msg.color.a = 1.0

        start_ = Point(x=start[0], y=start[1], z=0.0)
        end_ = Point(x=end[0], y=end[1], z=0.0)

        """ start_ = Point(x=self.best_line.start[0], y=self.best_line.start[1], z=0.0)
        end_ = Point(x=self.best_line.end[0], y=self.best_line.end[1], z=0.0) """

        msg.points.append(start_)
        msg.points.append(end_)

        self.line_pub.publish(msg)

    def _get_transform(self, parent: str, child: str, time: rclpy.time.Time = Time()) -> np.ndarray:
        """Returns the transformation matrix from parent to child frame."""
        transform = self.tf_buffer.lookup_transform(
            parent,
            child,
            time=time
        )
        rot = Rotation.from_quat([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]).as_matrix()
        trans = np.array([
            [transform.transform.translation.x],
            [transform.transform.translation.y],
            [transform.transform.translation.z]
        ])
        T = np.hstack((rot, trans))
        T = np.vstack((T, [0, 0, 0, 1]))
        return T
    
    def demo_line_detection(self): 
        ###
        ### DEMO LINE CLUSTERING
        ###
        # The folowing code was intended to be used to detect the pier for the MBZIRC demo
        if self.demo == True:
            # Filter the target line
            self.best_line = filter_lines(self, self.points, self.prev_angle)

            # If no line detected do not update variables
            if self.best_line == None: 
                return

            ### Avoid outliers to update kalman filter ###
            # This is done by saving the last 5 start and end line points and checking 
            # the distance between them, to avoid outliers updating kalman filter

            end_point_tracked = self.best_line.end
            start_point_tracked = self.best_line.start

            # Add the new measurement to the list
            self.measurements_x.append(end_point_tracked[0])
            self.measurements_y.append(end_point_tracked[1])

            # Ensure the list contains at most 5 measurements_x
            if len(self.measurements_x) > self.window_size:
                self.measurements_x.pop(0)  # Remove the oldest measurement
                self.measurements_y.pop(0)  # Remove the oldest measurement
            
            # Calculate the differences_x between the last 5 self.measurements_x
            differences_x = abs(np.diff(self.measurements_x))
            differences_y = abs(np.diff(self.measurements_y))
            
            # Calculate the median of the differences_x
            median_diff_x = np.median(differences_x)
            median_diff_y = np.median(differences_y)

            # Compare the difference between the current measurement and the previous one with the threshold
            if len(self.measurements_x) > 1:
                current_diff_x = abs(end_point_tracked[0] - self.measurements_x[-2])
                current_diff_y = abs(end_point_tracked[1] - self.measurements_y[-2])

                # If last measurement is between the accepted threshold update kalman filter
                if current_diff_x < median_diff_x * self.th_factor or current_diff_y < median_diff_y * self.th_factor:
                    self.tracked_end.update(np.array(end_point_tracked)[:,np.newaxis])
                    self.tracked_start.update(np.array(start_point_tracked)[:,np.newaxis])
                    #self._visualize_line()

                    if self.debug == True:
                        self.step_counter += 1
                        self.detected_points.append([self.step_counter, self.best_line.end[0], self.best_line.end[1]])
                        self.updated_points.append([self.step_counter, self.tracked_end.pos[0], self.tracked_end.pos[1]])

            else:
                # Update kalman filter until last 5 measurements have been saved
                self.tracked_end.update(np.array(end_point_tracked)[:,np.newaxis])
                self.tracked_start.update(np.array(start_point_tracked)[:,np.newaxis])

            # Calculate line angle wrt lidar x-axis (pointing forward)
            x1, y1 = self.best_line.start
            x2, y2 = self.best_line.end
            slope = (y2-y1)/(x2-x1)
            angle_x_axis = np.rad2deg(math.atan(slope))

            # Save the angle to compare it with the next detection
            self.prev_angle = angle_x_axis

            # Get the center of the line detected
            self.target_center = np.median(self.points, axis=1)
            marker = self.get_marker(self.target_center)
            #self.center_pub.publish(marker)


    def find_lines(self, points): 
        # Calculate the grid size based on the range of coordinates
        min_x, min_y = np.min(points, axis=1)
        max_x, max_y = np.max(points, axis=1)

        grid_size_x = int((max_x - min_x) / self.grid_resolution) + 1
        grid_size_y = int((max_y - min_y) / self.grid_resolution) + 1

        # Create an empty grid
        occupancy_grid = np.zeros((grid_size_x, grid_size_y), dtype=np.uint8)

        # Map laser scan points to the grid
        for x, y in points.T:  # Transpose 'points' to loop over columns
            x_index = int((x - min_x) / self.grid_resolution)
            y_index = int((y - min_y) / self.grid_resolution)
            occupancy_grid[x_index, y_index] = 255  # Set as occupied

        # Create image to be processed
        self.scan_image = occupancy_grid

        # Dilate image to join continuous points that define a line
        kernel = np.ones((self.dilate_kernel_size,self.dilate_kernel_size),np.uint8)
        self.scan_image = cv.dilate(self.scan_image,kernel,iterations = 1)

        # Classic straight-line Hough transform
        self.h_hl, self.theta_hl, self.d_hl = hough_line(self.scan_image)

    def detected_lines_filter(self, points, min_x, min_y): 
        ''' Iterate over the hough lines detected and filter them to get the ones desired'''
        
        # Get width and height of the scan image created
        width, height = self.scan_image.shape[1], self.scan_image.shape[0]
        
        #min_line_dist = 9999999

        # Obtain first and last point of each line detected in the image

        filtered_lines = []
        for _, angle, dist in zip(*hough_line_peaks(self.h_hl, self.theta_hl, self.d_hl)):
            # First and last line point in image frame
            y0 = (dist - 0 * np.cos(angle)) / np.sin(angle)
            y1 = (dist - (self.scan_image.shape[1] - 1) * np.cos(angle)) / np.sin(angle)
            x0 = 0
            x1 = self.scan_image.shape[1] - 1

            # Plot for debugging 
            """ ax[2].plot([x1, x0], [y1, y0])
            ax[2].set_xlim(0, width)
            ax[2].set_ylim(height, 0)  # Note: Y-axis is inverted in the typical image coordinate system """

            # Convert line points detected to laser scan frame
            p1 = [y0*self.grid_resolution+min_x, x0*self.grid_resolution+min_y]
            p2 = [y1*self.grid_resolution+min_x, x1*self.grid_resolution+min_y]

            # Create line with first and last point
            line = [p1, p2]

            # Visualize the line detected by hough lines (find_lines function)
            #line_msg = self._visualize_line(line[0], line[1], id=0)


            ### LINES DETECTED FILTER: Apply pca to estimate a line an select valid points
            ###                        within a thershold 

            ### 1st Filter: Get points that are below points_in_line_th_1 from the hough line

            # Get all laser points that belongs to the line within a threshold
            valid_points = self.points_on_line_with_threshold(points, line, self.points_in_line_th_1)

            # Convert points to an array
            valid_points_array = np.transpose(np.array(valid_points))
            
            # Continue with the loop if no valid points
            if len(valid_points_array) == 0: 
                continue
            
            ### 2nd Filter: Estimate previous line and apply a second threshold points_in_line_th_2

            # Estimate line from scan points using PCA
            valid_points_eq = est_line(valid_points_array)

            # Get inliers from that line within a threshold
            _, inliers, _ = all_points_valid(valid_points_eq, valid_points_array, threshold=self.points_in_line_th_2)
            
            # Update line cluster eliminating outliers
            valid_points_array = inliers[:2,:]

            # Continue with the loop if no valid points
            if valid_points_array.shape[1] == 0: 
                continue

            valid_points_eq = est_line(valid_points_array)

            # Get the points from the cluster
            x = list(inliers[0])
            y = list(inliers[1])
            z = list(inliers[2])
            points_target_line = []
            for i in range(len(x)): 
                points_target_line.append([x[i], y[i], z[i]])

            
            ### ORDER DETECTED POINTS AND GET THE BEGINNING AND THE END LINE POINTs
            if valid_points_array.shape[1] == 0: 
                continue
                
            first_point, last_point = self.get_first_last_line_points(valid_points_array, valid_points_eq)

            # Visualize the filtered line
            #line_msg = self._visualize_line(first_point, last_point)
            line_i = Line(eq= valid_points_eq, start=first_point, end=last_point, pts=valid_points_array)
            filtered_lines.append(line_i)

            # Get x axis line angle
            x1, y1 = first_point
            x2, y2 = last_point
            slope = (y2-y1)/(x2-x1)
            angle_x_axis = np.rad2deg(math.atan(slope))

            #lower, higher = self._order_line_endpoints(first_point, last_point)
            h = np.linalg.norm(last_point)
            l = np.linalg.norm(first_point)
            r = np.linalg.norm(last_point - first_point)
            curr_angle = np.arccos((r ** 2 + l ** 2 - h ** 2) / (2 * r * l))

            """ 
            line_i = Line(eq= valid_points_eq, start=first_point, end=last_point)

            line_i = Line(eq= valid_points_eq, start=first_point, end=last_point)

            if self.prev_ang == None: 
                self.prev_ang = angle_x_axis

            print(f"Prev angle: {self.prev_ang}")
            print(f"Curr angle: {angle_x_axis}")
            print(f"Ang diff:   {abs(angle_x_axis - self.prev_ang)}")

            if np.linalg.norm(line_i.center()) < min_line_dist and abs(angle_x_axis - self.prev_ang) < 45: 
                min_line_dist = np.linalg.norm(line_i.center())
                self.best_line = line_i """
            
        return filtered_lines
    
    def points_on_line_with_threshold(self, point_set, line, threshold):
        result = []
        for i in range(point_set.shape[1]):
            point = [point_set[0,i], point_set[1,i]]
            distance = distance_point_to_line(point, line)
            if distance <= threshold:
                result.append(point)
        return result
