import enum
import time
import traceback
from collections import deque
from typing import Optional, Tuple
import json

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2 #used to read points 
from std_msgs.msg import Int16, Bool

import math

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from mbzirc_interfaces.action import Dock

from mbzirc_dock.approach_policy import ApproachPolicy
from mbzirc_dock.line_detection import detect_lines, filter_lines
from mbzirc_dock.orbit_policy import OrbitPolicy
from mbzirc_dock.policy import Policy, PolicyAction, PolicyState, PolicyCanceledException, PolicyOutOfBoundsException
from mbzirc_dock.stop_policy import StopPolicy
from mbzirc_dock.target_policy import TargetPolicy
from mbzirc_dock.touch_policy import TouchPolicy
from mbzirc_dock.tracker import Tracker2D


class DockStage(enum.Enum):
    STOP = enum.auto()
    ENTER_ORBIT = enum.auto()
    ORBIT = enum.auto()
    APPROACH = enum.auto()
    TOUCH = enum.auto()


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
        
        ###
        ### PARAMETERS
        ###

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
        
        # DBSCAN clustering parameters
        self.epsilon = 2
        self.min_samples = 1

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
        self.line_thres_min = 0.7
        self.line_thres_max = 1
        self.angle_threshold = np.deg2rad(50) # 150
        self.prev_angle = None

        # Parameters to discard outlier measurements
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
        self.POI_coordinates = Point()
        self.POI_received = False
        #self.target_vessel_position = [[-1], [21]]
        
        # Initialize trajectory targets
        self.trajectory_targets = [
            np.array([[0.0, 0.0]]).T for _ in range(len(self.trajectory_relative))
        ]
        
        # USV mission state
        self.state = DockStage.STOP

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
        
        ###
        ### SUBSCRIBERS
        ###

        # Lidar subscriber
        """ self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self._laser_cb, 10) """
        self.laser_sub = self.create_subscription(
            PointCloud2, '/usv/slot6/points', self._laser_cb, 10)
        
        # Coordinator subscriber
        self.USV_to_shore_sub = self.create_subscription(Bool, 'USV_to_shore', self._USV_to_shore_callback, 10)
        self.POI_coords_sub = self.create_subscription(Point, 'POI_coordinates', self._POI_coords_callback, 10)

        ###
        ### PUBLISHERS
        ###

        # Publish velocity commands
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
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
        self._action_server = ActionServer(
            self, Dock, 'dock',
            self._execute_callback)
        
    def _usv_state_publisher(self): 
        # TO DO: Read current state and publish it
        test_state = 'WAIT_OBSTACLE'
        test_state_msg = self.USV_state_dict[test_state]

        USV_status_msg = Int16()
        USV_status_msg.data = test_state_msg

        self.usv_state_pub.publish(USV_status_msg)

    def _POI_coords_callback(self, POI_coords: Point) -> None:
        # TO DO: Read POI coordinates
        self.POI_coordinates = POI_coords
        self.target_vessel_position = [[self.POI_coordinates.x], [self.POI_coordinates.y]]
        self.POI_received = True
        self.get_logger().info(f"POI coordinates received: {self.POI_coordinates}")

    def _USV_to_shore_callback(self, USV_to_shore: Bool) -> None: 
        # TO DO: Read flag to come back to shore after picking up the boxes
        self.get_logger().info("Received message from the coordinator to come back to shore")

        self.get_logger().info(f"Message USV_to_shore: {USV_to_shore}")

    def _control_cb(self):
        print("CONTROL CALLBACK")
        """Control loop, called every 0.1 seconds. Executes current policy action."""
        try:

            # RVIZ clear path publisher
            if self.colision_status == True: 
                line_color = [1.0, 0.0, 0.0]
            else: 
                line_color = [0.0,1.0, 0.0]

            self.publish_line(start=[self.clear_path_width/2, 0.0], end=[self.clear_path_width/2, self.clear_path_length], id = 0, color = line_color)
            self.publish_line(start=[-self.clear_path_width/2, 0.0], end=[-self.clear_path_width/2, self.clear_path_length], id = 1, color = line_color)

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
        self.target_detected = False
        
        ###
        ### GET LASER SCAN FROM PC2
        ###

        # Get the center_line from the PointCloud2 and delete vessel points
        center_line = self._pc2_to_scan(pointcloud=pointcloud, ang_threshold=[1, -1])

        # Prepare points to process them
        self.points3d = np.array(center_line).T # 3xN points
        self.points = self.points3d[:2, :]      

        ###
        ### POINCLOUD CLUSTERING
        ###
        
        if self.POI_received == True:
            # Cluster the whole pointcloud looking for the POI and detecting obstacles. Manage obstacle avoidance
            self.pointcloud_clustering()

    def pointcloud_clustering(self): 
        ###
        ### DBSCAN CLUSTERING
        ###

        labels, label_idxs = self.dbscan_clustering()

        # Iterate over all clusters and find the target
        """ lines_array_msg = MarkerArray()
        lines_array_msg.markers = [] """
        updated_tracked_object = np.zeros(len(self.trackers_objects))

        # Itareate over DBSCAN mask to get target and obstacle clusters
        self.detect_target_and_obstacles(labels, label_idxs, updated_tracked_object)

        

        ### IF TARGET HAS BEEN OCLUDED, TRY TO FIND IT AGAIN

        if self.target_detected == False and self.tracker_target != None: 
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
                    self.target_detected = True
                    
            self.center_pub.publish(self.get_marker(self.tracker_target.pos.T, color = [0.0, 0.0, 255.0], scale = 0.3, id_ = 99999))

        ### REMOVE TRACKED OBJECTS THAT HAVE NOT BEEN DETECTED

        eliminate_tracked_objects_idxs = np.where(updated_tracked_object == 0)
        #print(f"Updated tracked objectss: {updated_tracked_object}")
        #print(f"Eliminate idxs: {eliminate_tracked_objects_idxs}")
        # Remove elements at specified indices using a loop
        if len(list(eliminate_tracked_objects_idxs[0])) > 0:
            for index in sorted(eliminate_tracked_objects_idxs, reverse=True):
                self.trackers_objects.pop(index[0])
                self.tracked_objects_points.pop(index[0])
                self.collision_objects.pop(index[0])

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
            #print(f" Object {k} tracked: {object_tracked.pos}")

        # If collision_status = True -> Collision detected / False -> Free path, no collisions
        self.colision_status = np.any(np.array(self.collision_objects))
        #print(f"Collision status: {self.colision_status}")
        self.get_logger().info("///////////////////////////////////////////////////////")
        self.get_logger().info(f"POI coordinates received: {self.POI_coordinates}")
        self.get_logger().info(f"Target tracked: {self.target_detected}")
        if self.target_detected == True: 
            self.get_logger().info(f"Target pose: {self.tracker_target.pos}")

        for k, object_tracked in enumerate(self.trackers_objects): 
            self.get_logger().info(f" Object {k} tracked: {object_tracked.pos}")
        self.get_logger().info(f"Colision status: {self.colision_status}")

    def detect_target_and_obstacles(self, labels, label_idxs, updated_tracked_object): 
        ''' Function that iterates over the mask created by DBSCAN and search 
            for the target as weel as the possible obstacles'''
        
        # Iterate over the DBSCAN mask
        for label_idx in label_idxs:
            # Get points from each cluster detected
            cluster_points  = self.points3d[:,np.where(labels == label_idx)[0]]

            # Get median from each cluster
            cluster_median_pos = np.median(cluster_points, axis=1)

            # Check if target is detected
            cluster_2d_pos = cluster_median_pos[0:2, None]

            ###
            ### CHECK IF THE CLUSTER IS THE TARGET OR AN OBSTACLE
            ###

            # If target tracker not initialized, calculate distance with target received
            if self.tracker_target == None:
                # self.target_vessel_position VARIABLE SHOULD BE RECEIVED AS A MESSAGE !!!!!!!!!!!!!!!!!!!!
                cluster_target_distance = np.linalg.norm(self.target_vessel_position - cluster_2d_pos)
            else: 
                # Calculate distance with target tracker if initialized
                cluster_target_distance = np.linalg.norm(self.tracker_target.pos - cluster_2d_pos[:,0])
            ##print(f"Object {label_idx}: {cluster_target_distance}")
            
            ### IF THE CLUSTER IS THE TARGET, UPDATE MEASUREMENT

            # If target detected from the beginning, start tracking it
            if cluster_target_distance < self.distance_threshold: 
                # Initialize target tracker
                self.target_detected = True
                if self.tracker_target == None: 
                    self.tracker_target = Tracker2D(
                        cluster_2d_pos[:,0], self.P0, self.R0, self.Q0, self.DT, self.mode)
                    self.tracked_target_points = cluster_points
                    self.center_pub.publish(self.get_marker(self.tracker_target.pos, color = [0.0, 0.0, 255.0], scale = 0.3, id_ = 99999))
                else: 
                    # Update target tracker
                    self.tracker_target.predict()
                    self.tracker_target.update(cluster_2d_pos)
                    self.tracked_target_points = cluster_points
                    
                    self.center_pub.publish(self.get_marker(self.tracker_target.pos.T, color = [0.0, 0.0, 255.0], scale = 0.3, id_ = 99999))
            
            ### IF THE CLUSTER IS AN OBSTACLE, UPDATE MEASUREMENT
            else: 
                
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
                
                # start tracking the object if the tracker list has not been initialized
                else: 
                    self.trackers_objects.append(Tracker2D(
                            cluster_2d_pos[:,0], self.P0, self.R0, self.Q0, self.DT, self.mode))
                    self.tracked_objects_points.append(cluster_points)
                    # Initialize tracked obstacle without collision to check it later
                    self.collision_objects.append(0)

            # Fill marker msg for cluster visualization in RVIZ
            """ for i in range(cluster_points.shape[1]): 
                x = cluster_points[0, i]
                y = cluster_points[1, i]
                marker = self.get_marker([x, y], self.cluster_colors[int(label_idx)], scale=0.1, id_=int(i))
                lines_array_msg.markers.append(marker)
        self.scan_pub.publish(lines_array_msg) """
    
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

        if self.tracked_start is not None:
            self._visualize_line()

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

    def _visualize_line(self) -> None:
        """Visualizes the tracked line."""
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'usv/sensor_6/sensor_link/lidar'
        msg.scale.x = 0.1
        msg.type = Marker.LINE_LIST
        msg.points = []
        msg.color.r = 1.0
        msg.color.g = 0.5
        msg.color.b = 0.5
        msg.color.a = 1.0

        start_ = Point(x=self.tracked_start.pos[0], y=self.tracked_start.pos[1], z=0.0)
        end_ = Point(x=self.tracked_end.pos[0], y=self.tracked_end.pos[1], z=0.0)

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