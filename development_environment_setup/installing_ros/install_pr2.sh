# Run install_ros_groovy.sh first.
echo "hddtemp hddtemp/daemon boolean false" | debconf-set-selections
apt-get install ros-groovy-pr2-desktop
