## Create A Colcon Workspace and Download MoveIt2 Source

mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src && git clone https://github.com/ros-planning/moveit2.git

wget https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos
vcs import < moveit2.repos
wget https://raw.githubusercontent.com/ros-planning/moveit2_tutorials/main/moveit2_tutorials.repos
vcs import < moveit2_tutorials.repos
