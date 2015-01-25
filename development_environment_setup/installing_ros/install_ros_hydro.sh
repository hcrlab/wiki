# Designed for Ubuntu 12.04.
# See http://wiki.ros.org/hydro/Installation/Ubuntu
echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list;
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -;
apt-get update;
apt-get -y install ros-hydro-desktop-full;
echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc;
rosdep init;
