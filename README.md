# group862_mini_project
This repository contains a ROS Noetic-based Gazebo simulation environment for the mini project of ROB8-Group 862 of Aalborg University, Spring 2025. Follow the guide below to install the necessary dependencies, set up the workspace, and launch the simulation.

-----------------------------------------------------------------------------------------------------------------------------------------------
## 🛠️ System Requirements<br>
#### ✅ Operating System: Ubuntu 20.04 <br>
#### ✅ ROS Distribution: Noetic <br>
#### ✅ Gazebo Version: Fortress (or compatible)
-----------------------------------------------------------------------------------------------------------------------------------------------
## 📌 1. Install ROS Noetic & Gazebo<br>
If ROS Noetic and Gazebo 11 are not installed, run the following:<br>
Step 1: Add ROS Repository
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
Add the key:
```
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
Step 2: Install ROS Noetic & Gazebo
```
sudo apt update && sudo apt install -y ros-noetic-desktop-full
```
Source ROS Noetic:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Step 3: Install Dependencies<br>
Run the following command to install all required packages:
```
sudo apt update && sudo apt install -y \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-msgs \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    libpcl-dev \
    python3-catkin-tools \
    python3-rospkg
```
-----------------------------------------------------------------------------------------------------------------------------------------------
## 📥 2. Clone the Repository<br>
Once your system is ready, clone the project repository:
```
cd ~
git clone https://github.com/OzanGaziYucel/group862_mini_project.git
cd group862_mini_project
```
-----------------------------------------------------------------------------------------------------------------------------------------------
## 🔧 3. Build the Workspace<br>
```
catkin_make clean  # Optional but recommended
catkin_make
source devel/setup.bash
```
To automatically source the workspace each time you open a new terminal, add this line to your ~/.bashrc
```
echo "source ~/group862_mini_project/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
-----------------------------------------------------------------------------------------------------------------------------------------------
## 🚀 4. Launch the Simulation<br>
To start the Gazebo simulation, run:
```
roslaunch mp_world mp_world.launch
```
This will: ✅ Start Gazebo<br>
✅ Load the custom world<br>
✅ Spawn the Kinect RGB-D camera<br>
✅ Publish TF frames<br>
-----------------------------------------------------------------------------------------------------------------------------------------------
