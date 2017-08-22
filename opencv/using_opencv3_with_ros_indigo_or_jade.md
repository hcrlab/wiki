ROS versions below Kinetic (e.g., Indigo, Jade) officially support OpenCV 2.4 but not 3.x.

## You must pick one or the other
If you want to use a feature of OpenCV 3, you will need to recompile `cv_bridge` and `image_geometry` from source.
These two packages are compiled with OpenCV 2 by default.
A consequence of this is that all packages in your workspace that use either `cv_bridge` or `image_geometry` will need to depend on OpenCV 3.
In other words, you can't have some packages in your workspace depend on OpenCV 2 and some on OpenCV 3, you must pick one or the other.
If you must have some packages depend on OpenCV 2 and some on OpenCV 3, then create separate workspaces for these packages.

## Instructions
- Install ROS's OpenCV 3.1: `sudo apt-get install ros-indigo-opencv3`
- Clone the correct version of [vision_opencv](https://github.com/ros-perception/vision_opencv) to your workspace, e.g., `cd ~/catkin_ws/src; git clone https://github.com/ros-perception/vision_opencv -b indigo`
- Update the following lines in your package's CMakeLists.txt:
  ```diff
  + find_package(OpenCV 3 REQUIRED)
  
  catkin_package(
  + DEPENDS OpenCV
  )
  
  include_directories(
  + ${OpenCV_INCLUDE_DIRS}
  )
  
  # Link with ${OpenCV_LIBRARIES} for each executable or library that uses OpenCV.
  target_link_libraries(my_library src/my_library.cpp
    ${catkin_LIBRARIES}
    + ${OpenCV_LIBRARIES}
  )
  ```
  
Now, build your package.
You should get no warnings.
If you see a warning like:
  
```
/usr/bin/ld: warning: libopencv_imgproc.so.3.2, needed by /usr/local/lib/libopencv_calib3d.so.3.2.0, may conflict with libopencv_imgproc.so.2.4
```
  
Then your project is simultaneously linking with OpenCV 2 as well as 3, which will likely cause your program to crash.
Check that you've correctly done the steps listed above.
