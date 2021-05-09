# unity_robotics_env
ROS_IP: 172.17.0.2
ROS_TCP_PORT: 10000
# Set up a docker container with ports 10000 and 5005, and mount your current directory as the unity robotics
# env. This assumes we are in the git repo base folder
cd Desktop/robotics
docker run -it --gpus=all --env NVIDIA_DISABLE_REQUIRE=1 -e TZ=Australia/Sydney -p 10000:10000 -p 5005:5005 -p 8888:8888 -v %cd%:/catkin_ws/src/robotics robotics 

## Run this
source /opt/ros/noetic/setup.bash
tmuxinator start -p catkin_ws/src/robotics/unity_robotics_env/mission_control.yml


## To access the jupyter notebook ( and enter the token if necessary)
http://127.0.0.1:8888




tmux kill-session -t mission_control


docker run -it --gpus all nvidia/cuda:11.2.2-devel-ubuntu20.04 /bin/bash






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
rosrun robotics_demo xyz_rpy_g_to_joints.py

source /catkin_ws/devel/setup.bash
rosrun robotics_demo core_logic.py

source /catkin_ws/devel/setup.bash
rosrun robotics_demo timer_loop.py

source /catkin_ws/devel/setup.bash
rosrun robotics_demo recorder.py

python3 /catkin_ws/src/unity_robotics_env/robotics_demo/scripts/server_endpoint.py
<!-- source /catkin_ws/devel/setup.bash
rosrun robotics_demo color_publisher.py -->
jupyter notebook --allow-root --ip=172.17.0.2 --port=8888 --no-browser

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



List of Todos!

1. Add the gripper
2. Control from VR - broadcast hand/controller position on the quat channel and convert it
3. Broadcast object positions
4. Integrate all reported objects
5. Attach camera to shoulder and wrist
6. Record ROS stuff for dataset
7. Save state and replay deterministically 


# (-0.8920287033165873, -0.22291259615495435, -1.910235427964787, -1.0097142591695119, 0.6779716923126915, 0.000988316049301452)23





tmuxinator
tqdm
ros noetic
tensorflow_probability
natsort
imagio
wandb
sudo apt-get -y install nano



export EDITOR='/usr/bin/nano'
export SHELL='/bin/bash'

make sure doors are not collison matrix approved with their base


# Unity tips n tricks
- Make use of layers, avoid collision with things that don't need to (a door and its cupboard), hide the controllers from view etc

apt-get update
apt-get install sudo -y
sudo apt-get install -y python3
sudo apt install python3-pip
pip3 install tensorflow
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt install ros-noetic-ros-base


sudo apt-get install --no-install-recommends \
    cuda-11-2 \
    libcudnn8=8.0.4.30-1+cuda11.2  \
    libcudnn8-dev=8.0.4.30-1+cuda11.2


sudo apt-get install libcudnn8=8.1.1.*-1+cuda11.2
sudo apt-get install libcudnn8-dev=8.1.1.*-1+cuda11.2

sudo ln -s /usr/local/cuda-11.2/targets/x86_64-linux/lib/libcusolver.so.11 /usr/local/cuda-11.2/targets/x86_64-linux/lib/libcusolver.so.10


sudo ln -s /usr/local/cuda-11.2/targets/x86_64-linux/lib/libcusolver.so.11 /usr/local/cuda-11.2/targets/x86_64-linux/lib/libcusolver.so.10
# Had to put this in bashrc fuckin hell
export LD_LIBRARY_PATH=/usr/local/cuda-11.2/lib64:/usr/local/cuda-11.2/lib64


gsutil -m cp "gs://lfp_europe_west4_a/saved_models/UnityB0_00/checkpoint" "gs://lfp_europe_west4_a/saved_models/UnityB0_00/ckpt-20.data-00000-of-00001" "gs://lfp_europe_west4_a/saved_models/UnityB0_00/ckpt-20.index" .