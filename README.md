![GitHub repo size](https://img.shields.io/github/repo-size/ab31mohit/platoon)


# Introduction
- This ros2 package is the implementation of platoon architecture & control being done as part of my BTP under [Prof. Arnab Dey](https://www.iitr.ac.in/~EE/Arnab_Dey).    

- This doc provides step by step process to setup this package & implement the results on the turtlebot3 hardware (also compatible with simulation).   

- This package implements platoon architecture using namespaces bringup files (to uniqely identify robots in the same ros2 environment).    

- The platoon control is implemented using the leader-follower trajectory control.     

- Static and dynamic obstacle avoidance is tested for simple environment by considering it as a lane changing problem for the current vehicle (robot).   

- Following terminology is used in this doc :   

  1. **SBC** --> The raspberrypi board on turtlebot3 hardware with installed ubuntu-22 server OS and ROS2 Humble.    

  2. **Remote PC** --> Your laptop of PC from where you will be running robots (via SSH) and running scripts.


## Repository Structure    
```bash 
├── DEBUGGING.md
├── LICENSE
├── platoon                                   # Meta package for containing other sub packages
│   ├── CMakeLists.txt
│   └── package.xml
├── platoon_control                           # Package to implement platoon control
│   ├── launch
│   │   └── platoon_control.launch.py         # Run leader-follower trajectory control for specified robots
│   ├── params
│   │   └── platoon_control.yaml              # Specify robots in the platoon
│   └── src
│       ├── formation_control_node.py         # Node to implement platoon control for a set of leader and follower robots
│       └── lane_changing.py                  # Node to implement goal navigation with obstacle avoidance
├── README.md                        
├── robot_bringup                             # Package to run the robots with namespaced platoon architecture
│   ├── launch
│   │   ├── bringup.launch.py                 # Main bringup file to run the robot
│   │   ├── ld08.launch.py   
│   │   └── robot_state_publisher.launch.py
│   ├── media                                 # Directory containing the data
│   ├── package.xml
│   ├── param                                 # Directory to initialize robot during bringup with namespaced topics
│   │   ├── burger.yaml
│   │   ├── waffle_pi.yaml
│   │   └── waffle.yaml
│   └── src
│       ├── robot_trajectory_node.py
│       └── update_ns_param.py                 # Python file to update the contents of param folder using the provided namespace
├── robot_teleop                               # Package to teleoperate a robot         
│   └── src
│       └── teleop_keyboard.py                 # Teleoperation node
├── TURTLEBOT3_HARDWARE_SETUP.md               
└── updated_turtlebot3_node                    # turtlebot3_node package with changes to implement custom odometry initialization
    │       ├── devices
    │       ├── odometry.hpp                   # Header file for specifying odom initialization functions and others
    │       ├── sensors
    │       └── turtlebot3.hpp
    ├── package.xml
    ├── param
    └── src
        ├── devices
        ├── odometry.cpp                       # cpp file for implementing odom initialization
        ├── sensors
        └── turtlebot3.cpp
```

## Requirements

- Working turtlebot3 hardware    
- Ubuntu 22.04 LTS Desktop (in Remote PC)     
- ROS2 Humble (in Remote PC)   

## Turtlebot3 Hardware setup     

- In case you need some guide on setting up turtlebot3 hardware with tricks, i've made one which you can follow:     

    [Turtlebot3 complete setup guide](/TURTLEBOT3_HARDWARE_SETUP.md)

## Install dependencies on Remote PC

- Update your debian packages :

  ```bash
  sudo apt update && sudo apt upgrade -y
  ```

- Make sure you have already setup the [`Turtlebot3 hardware & software`](/TURTLEBOT3_HARDWARE_SETUP.md) before going further.

- Install gazebo11-classic to simulate (in case you want to) your work & other turtlebot3 dependencies (necessary in all cases) :

  ```bash
  sudo apt install gazebo
  sudo apt install ros-humble-gazebo-*
  sudo apt install ros-humble-cartographer
  sudo apt install ros-humble-cartographer-ros
  sudo apt install ros-humble-tf-transformations
  sudo apt install ros-humble-tf2-tools
  sudo apt install ros-humble-navigation2
  sudo apt install ros-humble-nav2-bringup
  sudo apt install ros-humble-dynamixel-sdk
  ```

## Setting up the project  

### 1. Create a ROS2 workspace for this project in your remote PC :

- Open terminal and type these commands :

  ```bash
  cd ~
  mkdir -p ~/btp_ws/src
  cd ~/btp_ws/src
  git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
  git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
  git clone https://github.com/ab31mohit/platoon.git
  cd ~/btp_ws/src/turtlebot3/
  rm -rf turtlebot3_cartographer/ turtlebot3_navigation2/ turtlebot3_example/
  cd ~/btp_ws/
  colcon build --parallel-workers 1
  echo "source ~/btp_ws/install/setup.bash" >> ~/.bashrc
  ```    

    Make sure all the above cloned packages are built without any errors.       
    
    In some cases you might face some warnings after building. Just rebuild the workspace again and they'll be gone.    

    Also try to build the packages using the command `colcon build --parallel-workers 1` to build them one by one.     


### 2. Configure ROS2 environment in your remote PC :

- Include Cyclone DDS implementation for ROS2 Middleware

  ```bash
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  ```

- Export custom ROS_DOMAIN_ID for your entire project   

  ```bash
  echo "export ROS_DOMAIN_ID=13"
  ```
  I'm using `13` as my ros domain id. You can use anything between 0 and 255.    
  
  But make sure it is same for the SBC's of all the robots and your remote pc.

- Export the TURTLEBOT3_MODEL environment variable in your remote PC's `.bashrc` file:    

  ```bash
  echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
  ```

  Make sure to change the model, according to the hardware you are using. (like burger or waffle_pi)

- Export the TURTLEBOT3_NAMESPACE in your remote pc

  ```bash
  echo "export TURTLEBOT3_NAMESPACE=default_ns" >> ~/.bashrc
  ```     

  ***Note:***     

    - Here i'm using a dummy namespace ***default_ns*** for documentation purpose.    

    - Make sure to change this namespace according to your robot's name.                                                              

    - This namespace is used to connect to a specific robot (within the platooned ros2 environment) to access its topics/data.                         

    - For my work, i've used a specific pattern for thee namespaces which is ***TURTLEBOT3MODELInstance***.                        

    - For instance, the first burger will have namespace as *burger1* & third waffle_pi will have *wafflepi3* as its namespace to identify them easily in the platoon environment.  


### 3. Configure ROS2 environment in SBC (for each of the robots) :     

  Considering you have already configured your Hardware setup as mentioned [here](/TURTLEBOT3_HARDWARE_SETUP.md), follow the below steps to setup this project.    

- SSH into the Robot's RPI from your Ubuntu-22 terminal by connecting both to same local network :

  ```bash
  ssh user_name@ip_address_rpi
  ```    
  Change *user_name* and *ip_address_rpi* to the username & ip_address of your SBC's RPI.

  ***Note:***     

    - I would suggest you to use some tools such as termux or other to determine ip address of the SBC of your robot
      so that you don't need to connect your SBC to some monitor for its IP Address.

- Export the  ROS_DOMAIN_ID in RPI: (should be same for both, remote pc and SBC)

  ```bash
  echo "export ROS_DOMAIN_ID=13" >> ~/.bashrc
  ```
- Export namespace for your robot in RPI :    

  ```bash
  echo "export TURTLEBOT3_NAMESPACE=burger1" >> ~/.bashrc
  ```    
  
  Make sure to change this namespace according to your Robot's namespace.    
  
  Pattern for writing namespaces have been specified above in this doc.

- Clone this package in turtlebot3_ws in SBC of your robot :    

  ```bash    
  cd ~/turtlebot3_ws/src
  git clone https://github.com/ab31mohit/platoon.git
  ```

- Update the params in this package according to your robot namespace : 

  ```bash
  cd ~/turtlebot3_ws/src/platoon/robot_bringup/src/
  python3 update_ns_param.py
  ```    

  This file will replace the dummy namspace in [*burger.yaml*](/robot_bringup/param/burger.yaml) to the above specified one.      

  Obvioulsy it will change this file only if your robot model is burger as it also takes the robot model environment variable as input and changes that corresponding param file accordingly.
  
  This will ensure all the important topics of this robot are initiaised with a certain namespace (that you've set before).
  
  Do note that, running this file is a one time process for setting up SBC of your robot.    
  
  Also you don't need to run this on your remote pc, as the bringup file will be running from SBC and it will automatically set the namespaces according to what has been set in params.   


- Set initial pose of the robot :    

  You can initialize the odometry/pose of the robot by changing [*burger.yaml*](/robot_bringup/param/burger.yaml) file.      
    
  By default the initial transform between *world* and *default_ns/base_footprint* frame is [0, 0, 0] which is the starting value of odometry by default.     

  For platoon initialization, you will need to initialize all the robots at different positions in the same ros2 environment, so you can do that through this file.    


- Build the `turtlebot3_ws` in RPI : 
  
  ```bash
  cd ~/turtlebot3_ws/
  colcon build --parallel-workers 1
  ``` 
 
## Running the package 

  In this step you will bringup all the robots by connecting all of them along with your remote pc to same network.  

  You will need to know the ip addresses of all those robots so that you can SSH into them (for running their bringup files) from your Remote PC.     

### 1. Run bringup launch file for all the robots from your remote PC :    

- SSH into your robot's SBC from remote PC : 

  ```
  ssh rpi_username@rpi_ipaddr
  ```   
  Replace *rpi_username* & *rpi_ipaddr* to the username & ip address of your specific robot's SBC.    

- Run bringup file for the robot via above SSH connection :  

  ```
  ros2 launch robot_bringup bringup.launch.py
  ```   

  The log of this file should look like something like this  

    <div align="center">
      <img src="robot_bringup/media/ns_burger_bringup/ns_bringup_log.png" alt="Bringup log for burger2" />
    </div>   

  Here, I'm running ***burger*** (TURTLEBOT3_MODEL) with the namespace ***burger2*** (TURTLEBOT3_NAMESPACE).   

- Open a new tab in the remote PC & run : 
 
  ```
  ros2 topic list
  ```   
  If you're following everything correctly, it will show the topics started by bringup launch file of this robot in your remote PC.    
  
  The reason for this because of the same network and ROS_DOMAIN_ID between your remote PC and Robot's RPI.   
  
  It will show the topics something like this :    

    <div align="left">
    <img src="robot_bringup/media/ns_burger_bringup/ns_bringup_topics.png" alt="ROS2 topics for burger2" />
    </div>   

  Here all the topics of this robot are namespaced with ***burger2*** as i used this as the namespace for this robot except ***/tf*** and ***/tf_static***.     

  Now do the same bringup operation for all the robots.    

  After running all the robots together, the environment should look like this :     

  <!-- Testing platoon bringup -->
  |                          Hardware                            |                         Rviz                          |
  |--------------------------------------------------------------|-------------------------------------------------------|
  |![fig1](/robot_bringup/media/platoon/platoon_hardware.png)    |![fig2](/robot_bringup/media/platoon/platoon_rviz.png) |


### 2. Run the platoon control module :    

  - Platoon control part is implemented in the [platoon_control](/platoon_control/) package.    

  - This package contains a launch file named [platoon_control.launch.py](/platoon_control/launch/platoon_control.launch.py) file which runs the trajectory follower nodes for each pair of leader-follower robots within the platoon.     

  - The information of these pairs of leader-follower robots is specified within the [platoon_control.yaml](/platoon_control/params/platoon_control.yaml) file.   

  - Depending on the position of robots in your linea platoon, change the robot names accordingly in the [platoon_control.yaml](/platoon_control/params/platoon_control.yaml) file.    

  - Run the launch file for platoon control    

    ```bash  
    ros2 launch platoon_control platoon_control.launch.py
    ```       

  - This will initialize the relative distance between robots by usinig the current position of robots in the platoon and try to maintain that when of them moves forward.       

### 3. Run the goal node :   

  - Run the python node to move the first robot in the platoon (leader) to the goal while avoiding obstacles    

    ```bash   
    cd ~/btp_ws/src/platoon/platoon_control/src/   
    python3 lane_changing.py
    ```

## Extra notes     

- Use the [DEBUGGING.md](/DEBUGGING.md) file to understand and debug the files.      