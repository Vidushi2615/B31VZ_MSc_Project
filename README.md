# B31VZ_MSc_Project
## 1. Overview

This ROS package implements a complete pipeline for object detection, 3D localisation, autonomous navigation, and human-robot handover tasks using the Toyota Human Support Robot (HSR). It supports voice-commanded object fetching and returning, leveraging YOLOv8 for object detection, 3D point cloud localisation, and force-sensor-based interaction during handovers.
The system architecture integrates perception, manipulation, navigation, collision avoidance, and hand tracking modules into a coherent behaviour pipeline.

## 2.  Installation Guide 

Once you have ROS Noetic and HSR-related packages installed and running, you can proceed with the installation.

  ### 2.1. Clone the Repository ([Link to the Repository](https://github.com/Vidushi2615/B31VZ_MSc_Project.git))
  
  First, clone this repository into your ROS workspace src/ folder:
  
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/Vidushi2615/B31VZ_MSc_Project.git
  ```
  
  After cloning, build the workspace as explained below.
  
  ### 2.2. Install ROS Package Dependencies
  
  Use *rosdep* to automatically install missing ROS packages:
  
  ```bash
  cd ~/catkin_ws
  rosdep install \--from-paths src \--ignore-src -r -y
  ```

  If *rosdep* is not initialised yet, run:
  
  ```bash
  sudo rosdep init
  rosdep update
  ```
  
  ### 2.3. Install Required Python Packages
  
  Several Python packages are needed for YOLO detection, MediaPipe hand
  tracking, Kalman filtering, and computer vision:
  
  ```bash
  pip3 install ultralytics
  pip3 install mediapipe
  pip3 install filterpy
  pip3 install opencv-python
  pip3 install opencv-contrib-python
  pip3 install numpy
  pip3 install scipy
  ```

  Note:
  
  - Ultralytics provides YOLOv8 models.
  - Mediapipe is used for hand tracking.
  - Filterpy is used for Kalman filtering.
  
  ### 2.4. Install ROS-Specific Tools and Libraries
  
  Install these ROS packages via apt if they are missing:
  
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-vision-msgs
  sudo apt-get install ros-noetic-pcl-ros
  sudo apt-get install ros-noetic-tf2-ros
  sudo apt-get install ros-noetic-image-geometry
  sudo apt-get install ros-noetic-message-filters
  sudo apt-get install ros-noetic-cv-bridge
  sudo apt-get install ros-noetic-octomap-msgs
  sudo apt-get install ros-noetic-map-server
  sudo apt-get install ros-noetic-move-base-msgs
  sudo apt-get install ros-noetic-actionlib-msgs
  ```
  
  ### 2.5. Build the Workspace
  
  Finally, build your catkin workspace:
  
  ```bash
  cd ~/catkin_ws
  catkin_make
  source devel/setup.bash
  ```

  ### 3.3. scripts/

  #### 3.3.1. Core Nodes
  - **yolo_detector.py**: Runs YOLOv8 object detection on head RGB camera images and publishes 2D detections.
  - **object_marker_server.py**: Service server that transforms and publishes object position markers.
  
  #### 3.3.2. Behavioural Nodes
  
  - **listening_node.py**: Parses voice commands into actions and object names.
  - **main_controller.py**: High-level node executing full task pipelines based on the parsed voice commands (fetch, handover).
  
  #### 3.3.3. Navigation and Manipulation
  
  - **navigation_node.py**: Computes and publishes navigation goals at the target location.
  - **manipulation_service.py**: Moves the arm end-effector to the object\'s position and grasps.
  
  #### 3.3.4. Human Interaction and Handover
  
  - **hand_tracking_service.py**: Tracks the right human hand in 3D using MediaPipe and guides the end-effector to handover position.
  - **grasp_from_human_service.py**: Grasp object from human by detecting force changes.
  - **object_handover.py**: Releases object to human by monitoring wrist force drop.
  
  #### 3.3.5. Utilities
  
  - **clock_publisher.py**: Publishes /clock topic from joint_states to synchronise Gazebo simulation time.
  - **fix_octomap_centers.py**: Fixes timestamp and frame_id inconsistencies in OctoMap point clouds.
  - **voxel_filter_node.py**: Converts filtered pointcloud into collision map boxes for safe manipulation.

  
  ### 3.4. src/
  
  #### 3.4.1. Source Files
  
  - **tracker_with_cloud_node.cpp**: Provides core functionalities for projecting 2D detections into 3D space, clustering, and bounding box generation; used internally by other nodes but not a standalone executable.
  - **object_localisation_server.cpp**: Source file for the Action server handling 3D localisation of a specified object.
  - **get_hand_position_3d_server.cpp**: Source file implementing a service server to project detected hand positions into 3D space.
  
  ### 3.5. include/
  
  #### 3.5.1. Header Files
  
  - **tracker_with_cloud_node.h:** Header declarations associated with tracker_with_cloud_node.cpp.
  
  ### 3.6. launch/
  
  - **system.launch:** Main launch file to start the perception, manipulation, and handover systems.
  - **services.launch:** Launches all service nodes.
  - **collision.launch:** Sets up OctoMap-based collision environment using pre built static OctoMap.
  - **map.launch**: Launches map server for static mapping.
  - **world.launch**: Gazebo simulation world for the HSR robot.
  
  ### 3.7. config/
  
  Contains .yaml files for HSR configuration.
  
  ### 3.8. maps/
  
  Contains the 3D Octomap and 2D map file for lab configuration.
  
  ### 3.9. worlds/
  
  Contains the custom .world file used in gazebo simulation
  
  ### 3.10. models/
  
  Contains the yolov8n.pt file for Yolo detections
  
  ### 3.11. rviz/
  
  Contains the custom made .rviz file to subscribe to the correct topics for visual evaluation.

## 4. Main Nodes and Services 
<div align="center">

| Node | Description |
|:----:|:-----------:|
| /yolo_detector | 2D object detection using YOLOv8 |
| /object_localisation | Action server to localise objects in 3D |
| /get_object_marker | Service to get the odom-frame pose of the detected object |
| /manipulate_object | Service to move the arm to the target pose |
| /navigate_to_object | Service to move the base at the target location |
| /human_hand_tracking | Service to move the arm towards the human hand |
| /grasp_from_human | Service to grasp an object from a human hand |
| /release_object | Service to release the object to the human during handover |
</div>



## 5. Usage 

### 5.1. Launch Files

#### 5.1.1. system.launch 

- Launches the full robot system, including detection, localisation, manipulation, and handover services.
- If working on simulation: comment the clock publisher node and uncomment the world.launch include inside **system.launch**.
- Command: ``` roslaunch interactive_robot system.launch ```

#### 5.1.2. world.launch

- Launches the Gazebo simulation environment.
- Replace the .world file with your own custom Gazebo world in the launch file.
- Command: ``` roslaunch interactive_robot world.launch ```

#### 5.1.3. map.launch

- Launches the static map server to generate a new static OctoMap.
- After generating the map: ``` rosrun map_server map_saver -f ~/your_map_folder/your_map_name ```

#### 5.1.4. collision.launch 

- Loads and publishes a prebuilt Octomap for collision avoidance.
- Replace the .bt file with your own saved Octomap file.
- Command: ``` roslaunch interactive_robot collision.launch ```

### 5.2. Testing Each Node and Service

Once the system is up and running, you can use the following steps to test each node and service.

#### 5.2.1. /yolo_detector

- Type: Node
- How to check:
- Open RViz or *rqt_image_view*
- Subscribe to */yolo_detector/annotated_image*
- Expected output: Bounding boxes drawn around detected objects.
- Use ``` rostopic echo -n1 /detected_objects_2d``` to get the values of the bounding box

#### 5.2.2. /object_localisation 

- Type: Action

- How to call: ``` rostopic pub /object_localisation/goal interactive_robot/ObjectLocalisationActionGoal "object_name: 'bottle'"```
- How to check: Monitor */object_localisation/feedback* and */object_localisation/result* topics.
- Expected output: Returns 3D pose of the specified object.

#### 5.2.3. /get_object_marker

- Type: Service
- How to call: ``` rosservice call /get_object_marker "object_name: 'bottle'" ```
- How to check: Visualise */object_marker* topic in RViz (Marker message).
- Expected output: A sphere marker representing object position.

#### 5.2.4. /manipulate_object

- Type: Service
- How to call: Prepare a *geometry_msgs/PointStamped.*
```bash
rosservice call /manipulate_object "target:
  header:
    frame_id: 'odom'
  point:
    x: 1.0
    y: 0.5
    z: 0.7"
```
- How to check:
  - Observe the robot\'s arm moving towards the specified target.
  - Check */manip_target_marker* in RViz.

#### 5.2.5. /navigate_to_object

- Type: Service
- How to call:
- Send a target point:
```bash
rosservice call /navigate_to_object geometry_msgs/PointStamped "target:
  header:
    frame_id: 'odom'
  point:
    x: 2.0
    y: 1.0
    z: 0.0"
```
- How to check: Observe navigation goals being sent to */move_base*.
- Expected output: Robot base moves to the target location.

#### 5.2.6. /human_hand_tracking

- Type: Service
- How to call: ``` rosservice call /human_hand_tracking ```
- How to check: Visualise */tracked_hand_point* in RViz (published as *PointStamped*).
- Expected output: Robot arm approaches and moves near the detected right hand.

#### 5.2.7. /grasp_from_human

- Type: Service
- How to call: ``` rosservice call /grasp_from_human```
- How to check: Monitor the wrist force sensor feedback.
- Expected output: Robot grasps the object from human after detecting release.

#### 5.2.8. /release_object

- Type: Service
- How to call: ``` rosservice call /release_object ```
- How to check: Observe the gripper opening after detecting human pull.
- Expected output: Object released into human\'s hand and robot arm resets.

#### 5.2.9. /main_controller

- Type: Node
- Purpose: Executes the full sequence: object detection, navigation, grasping, and handover based on a given object and action.
- How to launch: ``` rosrun interactive_robot main_controller.py _object_name:='bottle' _action:='get bottle'```
- How to test:
  - Set the parameters object_name and action.
  - Action can be either \"get\" (to fetch an object) or \"keep\" (to place an object).
- Monitor:
  - Service calls and responses in terminal logs.
  - Robot behaviour: detection, navigation, manipulation, and handover sequence.
  - Verify visual feedback in RViz as mentioned in the above nodes.
- Expected outcome: The robot will execute the task in sequential order based on the action

## 6. Dependencies 

- ROS Noetic
- YOLOv8 (Ultralytics Python library)
- MediaPipe (for hand tracking)
- HSRB Interface SDK (hsrb_interface)
- PCL, OpenCV, FilterPy (Kalman filter), tf2_ros

## 7. Notes

- The robot operates in the odom frame for all transformations.
- A working OctoMap collision environment is needed for safe manipulation.
- The */listening_node* is provided for further integration of the system control based on voice commands. The /listening_node* would need some more work, and the */main_controller* would need modifications for complete integration.
