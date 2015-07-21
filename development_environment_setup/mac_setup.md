## Installing ROS on Mac

### Installing ROS Indigo
**Read them before start installing**
http://wiki.ros.org/indigo/Installation/OSX/Homebrew/Source
https://github.com/ros/rosdistro/issues/4597
https://gist.github.com/mikepurvis/9837958
These instructions are only tested on MacbookPro 11,3 and Mac OS X 10.9

```bash
# On your fresh OS X
# 1. Install XQuartz, Xcode and Command Line Tools (CLT) (see https://developer.apple.com/xcode/ or https://developer.apple.com/downloads/index.action# for details)
# 2. Install brew

brew update
brew install cmake

brew tap ros/deps
brew tap osrf/simulation  # Gazebo, sdformat, and ogre
brew tap homebrew/versions  # VTK5
brew tap homebrew/science  # others

rosinstall_generator desktop_full --rosdistro indigo --deps --wet-only --tar > indigo-desktop-full-wet.rosinstall
wstool init -j8 src indigo-desktop-full-wet.rosinstall

brew install cmake libyaml lz4
brew install boost --with-python
brew install opencv --with-qt --with-eigen --with-tbb
brew install pyqt

# Try to install dependent libraries...
rosdep install --from-paths src --ignore-src --rosdistro indigo -y

./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

# Now, reinstall RVIZ with hydro version
rosinstall_generator rviz --rosdistro hydro --tar >> indigo.rosinstall  # Version of rviz from Hydro
# Note that we are skipping installing dependency (rosdep install --from-paths src --ignore-src --rosdistro indigo -y)!
# If you run it, it will complain about not finding some deps, but seems to compile and run fine
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

# source from .bashrc file
```

## Installing ROS Hydro on Mac OS X 10.9
Follow instructions here: http://wiki.ros.org/hydro/Installation/OSX/Homebrew/Source

Notes:
* Make sure to use the **system python** not the **homebrew python**.
* I applied couple patches on
  * `robot_model` package: http://answers.ros.org/question/187042/unsuitable-pythonlibs-building-catkin-mac-os-x-1093/
  * `orocos_kinematics_dynamics ` package: http://answers.ros.org/question/94771/building-ros-on-osx-109-solution/
  * Note that applying only first patch might work.
* I couldn't building testing, so I had to use `-DCATKIN_SKIP_TESTING=1` for the `catkin_make` (read more about this flag [here](http://docs.ros.org/hydro/api/catkin/html/howto/format1/index.html).

### Conclusion
Visualization based on rviz and rqt works well and pkgs that mainly uses python works well. Use `-DCATKIN_BLACKLIST_PACKAGES="pkgs_to_not_build"` flags to exclude couple pkgs from the cmake build script.

If possible, use VMWare Ubuntu or dual boot Ubuntu

## General Mac Development Environment Setup

### Python
Using Python on Mac again is unnecessarily difficult.  I recommend installing fundamental python libraries with homebrew and use pip and virtualenv.  Below can links can be starting points of googling.

- http://docs.python-guide.org/en/latest/starting/install/osx/
- http://www.lowindata.com/2013/installing-scientific-python-on-mac-os-x/
- http://www.adamlaiacano.com/post/15606437039/setting-up-python-on-a-fresh-osx-install
- http://www.virtualenv.org/en/latest/

### tmux
I love tmux (so as the person who wrote [this page](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/tmux.md)). I highly recommend using it with iTerm2 (more about iTerm2 below) instead of terminal, iTerm2 supported tmux much better than terminal (as of 2014.04.27).  Again, seeding links are below.

- http://platypus.belighted.com/blog/2013/03/14/tmux-1/
- http://www.wikivs.com/wiki/Screen_vs_tmux
- http://dominik.honnef.co/posts/2010/10/why_you_should_try_tmux_instead_of_screen/
http://robots.thoughtbot.com/a-tmux-crash-course

If you are using tmux from Mac, you need more settings

- http://robots.thoughtbot.com/tmux-copy-paste-on-os-x-a-better-future

### iTerm2
- What is it? A better terminal for Mac OSX.
- [Install](http://www.iterm2.com/#/section/home)
- [Solarized colors](https://github.com/altercation/solarized/tree/master/iterm2-colors-solarized)
- [More color schemes](https://github.com/mbadolato/iTerm2-Color-Schemes)

### Are You Switching from Ubuntu to Mac?
Re-training your muscle memories for various programs can be tough.  Here are couple seeding links for bring your Ubuntu keyboard shortcuts back.

- http://lifehacker.com/225873/mac-switchers-tip--remap-the-home-and-end-keys
- http://superuser.com/questions/334019/switching-tabs-in-mac-terminal-with-ctrlpageup-pagedown
- http://blog.remibergsma.com/2012/02/18/home-key-in-osx-terminal/
- http://brettterpstra.com/2011/08/12/option-arrow-navigation-in-iterm2/
- https://chrome.google.com/webstore/detail/keyboard-shortcuts-to-reo/moigagbiaanpboaflikhdhgdfiifdodd

### Sharing Folders from Ubuntu
#### Quick start (Ubuntu 12.04)
From Ubuntu (Server)

1. `sudo apt-get install samba`
2. right click a folder you want to share from the ubuntu and click “Sharing Options"
3. Check "Share this folder"
4. `sudo smbpasswd -a username`

From Mac (Client)

1. `Cmd + K`
2. `smb://server_ip`
3. enter username and password
4. select the folder you have shared

#### More Readings
use samba: https://help.ubuntu.com/community/Samba/SambaServerGuide
then from client side, connect to the shared folders: http://askubuntu.com/questions/3815/how-to-share-files-between-ubuntu-and-osx
