# Run install_ros_hydro.sh first.
echo "hddtemp hddtemp/daemon boolean false" | debconf-set-selections
apt-get install ros-hydro-pr2-desktop
