![GitHub repo size](https://img.shields.io/github/repo-size/ab31mohit/platoon)


# Introduction
- This ros2 package is the implementation of platoon architecture & control being done as part of my BTP under [Prof. Arnab Dey](https://www.iitr.ac.in/~EE/Arnab_Dey).    

- This doc provides step by step process to setup this package & implement the results on the turtlebot3 hardware (also compatible with simulation).   

- This package implements platoon architecture using namespaced bringup files (to uniquely identify robots in the same ros2 environment).    

- The platoon control is implemented using the leader-follower trajectory control approach.     

- Static and dynamic obstacle avoidance is tested for simple environment by considering it as a lane changing problem for the current vehicle (robot).   

- Following terminology is used in this doc :   

  1. **SBC** --> The raspberrypi board on turtlebot3 hardware with installed ubuntu-22 server OS and ROS2 Humble.    

  2. **Remote PC** --> Your laptop of PC from where you will be running robots (via SSH) and running scripts.


## Repository Structure    

```bash 
├── DEBUGGING.md                       # debug instructions          
├── LICENSE                            # license information
├── PROJECT_GUIDE.md                   # complete project understanding
├── README.md                          # main guide to setup & implement project
├── TURTLEBOT3_HARDWARE_SETUP.md       # turtlebot3 hardware setup guide
├── media/                             # pictures/plots used as contents
├── platoon                            # meta package for containing other sub packages
│   ├── CMakeLists.txt
│   └── package.xml
├── platoon_control                    # package to implement platoon control
│   ├── CMakeLists.txt
│   ├── launch
│   │   └── platoon_control.launch.py  # run platoon control nodes for all the robots
│   ├── package.xml
│   ├── params
│   │   └── platoon_control.yaml       # specify platoon structure
│   └── src
│       ├── formation_control_node.py  # platoon control node
│       └── lane_changing.py           # goal navigation node
├── robot_bringup                      # package to bringup robots with namespaces
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── bringup.launch.py          # Main bringup file to run the robot
│   │   ├── ld08.launch.py
│   │   └── robot_state_publisher.launch.py
│   ├── package.xml
│   ├── param                          # file to namespace nodes & start position    
│   │   ├── burger.yaml
│   │   ├── waffle_pi.yaml
│   │   └── waffle.yaml
│   └── src
│       ├── robot_trajectory_node.py
│       └── update_ns_param.py         # file to replace dummy namespace in param folder
├── robot_teleop                       # package to teleoperate a robot 
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── teleop_keyboard.py         # teleoperation node
└── updated_turtlebot3_node            # turtlebot3_node package with updates to initialize custom odometry
    ├── CMakeLists.txt
    ├── include
    │   └── updated_turtlebot3_node
    │       ├── devices/
    │       ├── odometry.hpp
    │       ├── sensors/
    │       └── turtlebot3.hpp
    ├── package.xml
    ├── param
    │   ├── burger.yaml
    │   ├── waffle_pi.yaml
    │   └── waffle.yaml
    └── src
        ├── devices/
        ├── node_main.cpp              # main executable node of this package
        ├── odometry.cpp               # file to initialie and update odometry
        ├── sensors/
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
      <img src="media/ns_burger_bringup/ns_bringup_log.png" alt="Bringup log for burger2" />
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
    <img src="media/ns_burger_bringup/ns_bringup_topics.png" alt="ROS2 topics for burger2" />
    </div>   

  Here all the topics of this robot are namespaced with ***burger2*** as i used this as the namespace for this robot except ***/tf*** and ***/tf_static***.     

  Now do the same bringup operation for all the robots.    

  After running all the robots together, the environment should look like this :     

  <!-- Testing platoon bringup -->
  <!-- |                          Hardware                            |                         Rviz                          |
  |--------------------------------------------------------------|-------------------------------------------------------|
  |![fig1](/media/platoon/platoon_hardware.png)    |![fig2](/media/platoon/platoon_rviz.png) | -->

    <table>
        <thead>
            <tr>
            <th style="text-align:center;">   Hardware bringup  </th>
            <th style="text-align:center;">Rviz data</th>
            </tr>
        </thead>
        <tbody>
            <tr>
            <td style="text-align:center;">
                <img src="media/platoon/platoon_hardware.png" alt="fig1" />
            </td>
            <td style="text-align:center;">
                <img src="media/platoon/platoon_rviz.png" alt="fig2" />
            </td>
            </tr>
        </tbody>
    </table>

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

<!-- 
## Results    

  The video results for different scenarios are as follows :      


  1. [Platoon control for a single set of leader follower](https://drive.google.com/file/d/17kOYZ_MorR6m7O9nQKXy5yPtRre6W99N/view?usp=sharing) :     

      - in this case, the leader was teleoperated from the keyboard using [teleop_keyboard.py](robot_teleop/src/teleop_keyboard.py) node.  
      - the follower was running the [platoon_control.launch.py](platoon_control/launch/platoon_control.launch.py) file.     

  2. [Platoon control with three robots and a static obstacle](https://drive.google.com/file/d/1sQWRfjM5dRVhDmP_qC5Lvjde848lwWqA/view?usp=sharing) :    

      - in this case, the leader was given some goal and it was running the [lane_changing.py](platoon_control/src/lane_changing.py) node.    
      - the other 2 robots were running the [platoon_control.launch.py](platoon_control/launch/platoon_control.launch.py) file and the [platoon_control.yaml](platoon_control/params/platoon_control.yaml) file specified the leader-follower sets.     

  3. [Platoon control with 2 robots and dynamic obstacle](https://drive.google.com/file/d/1nnj8WbNjm0aAW5XdpUUlXpR5N-95k62B/view?usp=sharing) :    

      - in this case, leader was given some goal and it was running the [lane_changing.py](platoon_control/src/lane_changing.py) node.
      - the follower robot was running the [platoon_control.launch.py](platoon_control/launch/platoon_control.launch.py) file which was triggering the [lane_changing.py](platoon_control/src/lane_changing.py) node upon dynamic obstacle detection. 
 -->

## Results    

  The video results for different scenarios are as follows :      


  1. ***Platoon control for a single set of leader follower*** :    

      https://github.com/user-attachments/assets/6869dedf-8055-48f2-9438-9c7f2276f6e0     


      - in this case, the leader was teleoperated from the keyboard using [teleop_keyboard.py](robot_teleop/src/teleop_keyboard.py) node.  
      - the follower was running the [platoon_control.launch.py](platoon_control/launch/platoon_control.launch.py) file.     

  2. ***Platoon control with three robots and a static obstacle*** :    

      https://github.com/user-attachments/assets/f6b6f02d-6fed-4532-8af5-18c2949dbc56

      - in this case, the leader was given some goal and it was running the [lane_changing.py](platoon_control/src/lane_changing.py) node.    
      - the other 2 robots were running the [platoon_control.launch.py](platoon_control/launch/platoon_control.launch.py) file and the [platoon_control.yaml](platoon_control/params/platoon_control.yaml) file specified the leader-follower sets.     

  3. ***Platoon control with 2 robots and dynamic obstacle*** :    

      https://github.com/user-attachments/assets/01851d4f-7867-4873-9504-d4c33c15d56f

      - in this case, leader was given some goal and it was running the [lane_changing.py](platoon_control/src/lane_changing.py) node.
      - the follower robot was running the [platoon_control.launch.py](platoon_control/launch/platoon_control.launch.py) file which was triggering the [lane_changing.py](platoon_control/src/lane_changing.py) node upon dynamic obstacle detection. 


## Extra Notes     

- Use the [DEBUGGING.md](/DEBUGGING.md) file to understand and debug the files.      
- Use the [PROJECT_GUIDE.md](/PROJECT_GUIDE.md) file to understand the entire Project structure and how every file is working.
