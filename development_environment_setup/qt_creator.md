# QtCreator
Qt Creator: it is IDE integrates CMakeLists.txt very well. This is quite convenient for ROS since it uses cmake extensively. However, after catkin, you have to do little heck to use Qt Creator with catkin build system:
- http://answers.ros.org/question/67244/qtcreator-with-catkin/
- http://answers.ros.org/question/73468/how-to-import-ros-project-c-file-into-qt-creator-and-compile-them/

also have to use this trick to make it work with x-forwarding:
https://bugreports.qt.io/browse/QTCREATORBUG-11173

Here is a bit older note about Qt Creator from ROS:
http://wiki.ros.org/IDEs#QtCreator

Also Qt Creator's indentation is little different with ROS, you can check here to fix that:
http://qt-project.org/forums/viewthread/9941
http://doc.qt.digia.com/qtcreator-extending/
