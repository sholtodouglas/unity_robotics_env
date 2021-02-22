# unity_robotics_env
ROS_IP: 172.17.0.2
ROS_TCP_PORT: 10000
# Set up a docker container with ports 10000 and 5005, and mount your current directory as the unity robotics
# env. This assumes we are in the git repo base folder
cd Desktop/unity_robotics_env
docker run -it  -p 10000:10000 -p 5005:5005 -v %cd%:/catkin_ws/src/unity_robotics_env robotics
# Make and source the env
tmux
cd catkin_ws
catkin_make
source /catkin_ws/devel/setup.bash
roscore

# Create a new pane with tmux


source /catkin_ws/devel/setup.bash
rosparam load /catkin_ws/src/unity_robotics_env/configs.yaml
rosrun robotics_demo server_endpoint.py

source /catkin_ws/devel/setup.bash
rosrun robotics_demo color_publisher.py

rosrun robotics_demo xyz_rpy_g_to_joints.py
rosrun robotics_demo consolidate_state.py

# Unity 
ROS IP Address - 127.0.0.1
ROS Port - 10000

# Find IP address with cmd > ipconfig, take the IPV4 address
ipconfig 

Override Unity IP Address 127.0.0.1
Unity Port 5005

# If you make a new ros node - make it executale with chmod +x and put it in the CMakeLists.txt
# For any messages we want to publish/subsribe to from unity,
# use the Unity Robotics > Generate ROS messages to generate C# scripts


netstat -ano | findstr :10000


docker run -it --net=host -v %cd%:/catkin_ws/src/unity_robotics_env robotics



‘docker commit ID robotics’
taskkill /PID 12140 /F


####### URDF Importing

https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/urdf_importer/urdf_tutorial.md

https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/quick_setup.md


# Creating the unity env
1. Create the env from unity hub
2. Get the ROS TCP connector - https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md
3. Create plane
3.5. https://github.com/Unity-Technologies/URDF-Importer import URDF package
4. Create a table at 0,0,0 https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/1_urdf.md
5. Open the Physics Project Settings (in the top menu bar, Edit > Project Settings > Physics) and ensure the Solver Type is set to Temporal Gauss Seidel. This prevents erratic behavior in the joints that may be caused by the default solver. (https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/pick_and_place/1_urdf.md)
6. Import the URDF as Y axis up, set base link to immovable, ensure URDF has a base link and no links with only inertial properties
7. Use GUI horizontal sliders and xDrive to test movement
8. Now, creating msgs!