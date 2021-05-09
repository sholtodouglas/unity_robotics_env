# Setting up this environment to work with learning from play
There are three steps
 - Create a master folder called 'robotics' - I have it on desktop. TODO: Create a configurable 'lfp dir path'. 
 -  Clone https://github.com/sholtodouglas/learning_from_play inside the folder
 - Clone this repo inside the folder for the env and interfacing code. As Unity packages, install the https://github.com/Unity-Technologies/ROS-TCP-Connector (use the files provided) and https://github.com/Unity-Technologies/URDF-Importer (only needed if you want to change the arm). The dev version of ROS-TCP-Connecter we are using can be found in the Unity env's assets folder - install it directly from disk rather than using git URL. For the env, you'll need to pop the complete home assets interior package in there. Use this drive link https://drive.google.com/drive/folders/1TAvZbUMV5q9_phzpmqfOqVl6jAIDJDxE?usp=sharing
 - Download the docker image sholto/robotics:latest from https://hub.docker.com/repository/docker/sholto/robotics
 - Optional - download the test set https://drive.google.com/drive/folders/10jdoW8XLX7zJlKsVD5fKg8NFZfjbUn8w?usp=sharing. This will let you play out data.




## Running everything
```
cd Desktop/robotics
docker run -it --gpus=all --env NVIDIA_DISABLE_REQUIRE=1 -e TZ=Australia/Sydney -p 10000:10000 -p 5005:5005 -p 8888:8888 -v %cd%:/catkin_ws/src/robotics sholto/robotics 
```
## Once in the container, run this
```
source /opt/ros/noetic/setup.bash
tmuxinator start -p catkin_ws/src/robotics/unity_robotics_env/mission_control.yml
```

## To access the jupyter notebook (enter the token  - you'll see it in the tmuxinator pane)
http://127.0.0.1:8888


See /robotics_demo/scripts/ControlCentre.ipynb for an example of how to reset the env and command the arm. 





