sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
&& sudo apt install curl \
&& curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
&& sudo apt-get update \
&& sudo apt-get install ros-noetic-desktop-full ros-noetic-moveit \
&& sudo apt-get install python3-rosdep \
&& sudo rosdep init \
&& rosdep update \
&& sudo apt-get install ros-noetic-webots-ros 