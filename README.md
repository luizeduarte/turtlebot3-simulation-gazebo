# Simulating Turtlebot3 using Gazebo!

### 1. Installing ROS2.

  To begin, install ROS2 following the steps from the official [ROS2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
). The distribution Humble was chosen.

   Other packages that could be useful in the future:
   ```
   $ sudo apt install ros-humble-cartographer
   $ sudo apt install ros-humble-cartographer-ros
  ```

### 2. Installing Nav2.
  But first, what is Nav2? 
  It is a package that provides navigation-related functionalities for robots. It's used for path planning, obstacle avoidance, and other navigation tasks in robotics. 
  ```
   $ sudo apt install ros-humble-navigation2
   $ sudo apt install ros-humble-nav2-bringup
   ```

### 2. Installing Gazebo.

Download the Gazebo Simulator:
```
$ sudo apt install ros-humble-gazebo-*
```

### 3. The turtlebot3 packages.

Create your workspace for turtlebot3 and clone the necessary files:
```
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws/src/
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Compile your workspace: 
```
$ cd ~/turtlebot3_ws && colcon build --symlink-install
$ source ~/turtlebot3_ws/install/setup.bash
```
### 4. Setting up your workspace.

To configure your workspace for use, type in each terminal you open:
(It is also possible to use the waffle version instead of the burger).
```
$ export TURTLEBOT3_MODEL=burger  
$ source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
$ source /opt/ros/humble/setup.bash
$ source ~/turtlebot3_ws/install/setup.bash
$ export ROS_DOMAIN_ID=30 #TURTLEBOT3
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```
Adding the commands to the bashrc is possible, avoiding typing them again at each terminal.

### 5. Running a turtlebot3 simulation in Gazebo using Nav2!
```
$ ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
(It will launch your robot in the turtlebot3_world)

### 6. Navigating with the turtlebot.
By default, Nav2 waits for you to give it an approximate starting position since the robot has no idea where it is. Next, click the “Navigaton2 Goal” button and choose a destination. This will call the BT navigator to go to that goal through an action server.

To control the robot with your keyboard, open a new terminal and execute the following command:
```
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### ! For extra info, access the [Turtlebot3 e-manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/).

### 7. Possible problems.
* When compiling your workspace using colcon build, the following message could show up: 
“setup.py install is deprecated. Use build and pip and other standards-based tools”. In this case, your Python version should be 58.2.0 or older, type:
```
$ pip install setuptools==58.2.0.
```

* In case your computer warns that a turtlebot3 package is missing after following all the steps, install:
```
$ sudo apt install ros-humble-turtlebot3*
```

* In case Gazebo shows an error message while running the simulation, type the following command before trying to fix it:
```
$ killall -9 gzserver
```
A known issue with this version of Gazebo is that its server side ends up not shutting down properly after a run.
