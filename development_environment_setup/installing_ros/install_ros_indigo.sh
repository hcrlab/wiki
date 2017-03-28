# Designed for Ubuntu 14.04.
# See http://wiki.ros.org/indigo/Installation/Ubuntu
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list;
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
apt-get update;
apt-get -y install ros-indigo-desktop-full;
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc;
rosdep init;
