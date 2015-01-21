# Designed for Ubuntu 12.04
# Run http://wiki.ros.org/groovy/Installation/Ubuntu
echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list;
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add -;
apt-get update;
apt-get -y install ros-groovy-desktop-full;
echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc;
