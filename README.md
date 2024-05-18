# Human Robot Interaction Outreach Project with Augmented Reality Headset

## System Overview
An overhead camera streams an aerial live video of a ground robot to a hologram on Hololens 2, where the user would have three options to move the ground robot with the holographic interface. The first option is to direct the robot to one desired location by air-tapping on the holographic interface. Alternatively, the user would select waypoints in a sequential order in a pre-designed route. Finally, the user could also click buttons that would make the robot perform specified tasks such as ’spin’ or ’advance one meter’. The interface displays the past and predicted route of the robot on the hologram to reduce cognitive load and improve user experience.

The hologram displays the past and predicted optimal trajectories as blue dashed and solid lines respectively. When the user selects a target point by air-tapping on the interface, a red dot will be displayed and when the rover reaches the target coordinate, the red dot would disappear.

## Hardware Components/Prerequisites
The system comprises 
- a Jetson Xavier as the processing unit of the drone
- an Intel RealSense Camera D435  that will be strapped onto the drone. During the testing phase, the camera is strapped onto a beam that faces the ground.
- a Microsoft Hololens that the user will interact with
- a ground station computer to monitor and interact with the rover
- a ground rover in Differential Drive mode
  - A Raspberry Pi as the processing unit for the rover
  - [Orange Cube](https://docs.px4.io/main/en/flight_controller/cubepilot_cube_orange.html)
- [Vicon Coordinate System](https://www.vicon.com/) that provides real time information about the pose of the robot. Note that a minimum of three 'Vicon Pearls' would be needed.

![alt text](rover.png)

## Software Prerequisites
**On Xavier**:
- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/foxy/index.html)
- OpenCV2 and Numpy
- [Realsense libraries](https://github.com/IntelRealSense/librealsense)
- [Vicon Bridge](https://github.com/dasc-lab/ros2-vicon-bridge) `git clone https://github.com/dasc-lab/ros2-vicon-bridge.git`

**On Rover**:
- Install docker on Raspberry PI 
- [PX4-Autopilot-Quad](https://github.com/dev10110/PX4-Autopilot-Quad/tree/rover) flashed onto Orange cube
  - **Clone 'Rover' brach**: `git clone -b rover https://github.com/dev10110/PX4-Autopilot-Quad.git`
  - in folder run the following command:
     - `make cubepilot_cubeorange_dasc`
- [QGC software](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) installed on ground station computer
- [ground station](https://github.com/dasc-lab/rover_groundstation_ros2_jumpstart) `git clone https://github.com/dasc-lab/rover_groundstation_ros2_jumpstart.git`

## System and Terminal Setup
### Vicon
* Initializes Vicon coordinate frame and start tracking your robot.
  * At least three Vicon dots will need to be placed on the robot for the tracker to initialize the robot in the system.
  * Initialize the robot in the system by selecting the three Vicon dots attached to the robot in the 'Object' tab of the interface and enter a name for it.
  * Save the object as **public**
  * Click 'Track' to start broadcasting the pose and coordinate of the robot through ROS2
### Xavier
* Set up Vicon Bridge
  * Open up a terminal window and in your project `/ros2_ws/src` folder,  enter `source install/setup.bash` Note: **It is recommended that you put this line in .bashrc file**
  * Navigate to the directory that holds `all_segments.launch.py` and modify the IP address to Vicon computer IP address.
  * Run Vicon bridge with the command `ros2 launch vicon_bridge all_segments.launch.py`
  * (Optional) In a new terminal, run the command `ros2 topic list` to see the current available topics
  * In a new terminal, run the command `ros2 topic echo /your_topic_name' to visualize the broadcast information
* Set up PX4_msgs
   * Download px4_msgs_main and move it to `~/ros2_ws/src`
   * `colcon build --symlink-install`, for more info, see [colcon_build](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
### Rover
* Set up Raspberry PI (on ground station computer)
  * SSH into Raspberry PI onboard the rover (may take minutes for Raspberry Pi to boot up)
  * `cd rover_px4_ros2_jumpstart`
  * `docker compose build` (first time)
  * `docker compose up -d`
  * `docker exec -it rover_px4_ros2_ bash` (use tab-complete)
  * `ros2 launch all_launch px4.launch.py`
  * launch QGC software
* On local ground station computer
  * Navigate to `rover_groundstation_ros2_jumpstart`
  * `docker compose build` (first time)
  * `docker compose up -d`
  * `ros2 launch ground_station gs.launch.py` (tab complete)
  * If an error occurs, open up a new window in the terminal and type in command  `xhost +` and try the prior steps
## Camera Calibration
It is recommended that you calibrate the camera for better coordinate transformation results. We will be utilizing apriltags and `cv2.solvePnP()` to calibrate the camera extrinsic matrix. We printed out 6 apriltags of the same family and laid them on known coordinates of the ground. Using the apriltag library provided by python, we can obtain the pixel coordinates of the apriltag points and find the correspondence. `cv2.solvePnP()` would output the extrinsic matrix
## Socket Communication
In python script `xavier_node_multithreading_dashed.py`, change the variable **self.HOST** to the IP address of your Xaiver.
