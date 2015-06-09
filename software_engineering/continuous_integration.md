# Continuous integration with ROS

## What and why
Continuous integration makes it possible to build your repository and run tests automatically for every commit. Continuous integration systems start with a fresh VM or container, install your code from scratch, builds it, and runs other scripts like tests. Because the system starts with a fresh VM every time, this gives you confidence that others can repeat the build.

Examples of things you might want to run automatically:
- Build: hopefully your code compiles before you push it! However, sometimes catkin works for one person but not others, usually because of missing dependencies in CMakeLists.txt.
- Test: run unit tests automatically and find out exactly which commit broke the tests.
- Lint: run a linter or some kind of static analysis.
- Generate documentation: run Sphinx or whatever other documentation generator you're using.
- Deploy: deploy generated documentation to a website, or deploy built Debians.

## How to set up Travis
If your project is open-source on Github and you're running on a ROS distro that supports Ubuntu 12.04 (e.g., Groovy or Hydro), then it's easy to do continuous integration with [Travis](https://travis-ci.org/).

1. Go to [Travis](https://travis-ci.org/) and log in with your Github account.
2. Go to your profile and enable Travis for the repository you want to set up.
3. Travis will trigger automatically when you push to the repository.

### Configure the Travis build
To configure what Travis does when you push, add a .travis.yml file in the root of your repository. Below is a sample .travis.yml for ROS. Replace YOUR_REPO with your repo:
```yaml
# General setup
# -------------
language: python
python:
    - "2.7"
# Allows the python virtualenv to use apt-get installed packages, which
# is essential (as ROS recommends this and pip doesn't seem to contain
# all packages, or contains them with errors).
virtualenv:
    system_site_packages: true
# Allow caching of debian (apt-get) packages. This means they're cached
# on Travis, so we still have to download/install them, but it will be
# faster than going to the ubuntu repositories.
cache: apt
# Before anything, see if we can get the mongoDB troubles out of the way.
# Note that this is a Travis-CI specific problem; this is not needed in
# general.
before_install: # Use this to prepare the system to install prerequisites or dependencies
    # Define some config vars
    - sudo apt-get --purge remove mongodb-10gen postgresql-9.2 postgresql-contrib-9.2 postgresql-9.3 postgresql-contrib-9.3
    - export ROS_DISTRO=hydro
    # Settings to make installing script more general.
    - export ROS_CI_DESKTOP=`lsb_release -cs`  # e.g. 'precise'

notifications:
  email: false

# Commands to install dependencies
# --------------------------------
install:
    # Install ROS
    - echo "deb http://packages.ros.org/ros/ubuntu $ROS_CI_DESKTOP main" | sudo tee /etc/apt/sources.list.d/ros-latest.list
    - wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    - sudo apt-get update -qq
    - sudo apt-get install -qq -y python-catkin-pkg python-rosdep python-wstool ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-ros
    - source /opt/ros/$ROS_DISTRO/setup.bash
    # Setup rosdep
    - sudo rosdep init
    - rosdep update
    # Create workspace.
    - mkdir -p ~/catkin_ws/src
    - cd ..; mv YOUR_REPO ~/catkin_ws/src
    - cd ~/catkin_ws/src
    - catkin_init_workspace
    # Install dependencies
    - cd ~/catkin_ws/
    - rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
    # Build
    - catkin_make

# Command to run tests
# --------------------
script:
    - catkin_make run_tests # Run tests using catkin
    - rostest YOUR_REPO test.launch # Run tests using rostest
    - pep8 src/*.py test/*.py # Linter
```

See the [Travis docs](http://docs.travis-ci.com/) for more information.
