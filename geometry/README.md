# Common geometry tasks

## Interpreting rotation matrices
A rotation matrix is the easiest way to understand a rotation.
Suppose a rotation matrix describes the rotation from frame A to frame B.
The columns of the rotation matrix are frame B's x, y, and z unit vectors, expressed in frame A's coordinate system.

For example, the following rotation matrix expresses a roll of 30 degrees:
```
| 1 0     0     |
| 0 0.866 -0.5  |
| 0 0.5   0.866 |
```

The first column shows that the x axis is unchanged.
The second column shows that frame B's y axis is mostly still in the y direction, but it now also has a z component.
The third column shows that frame B's z axis is mostly still vertical, but also tilts to the right a bit.

## Convert quaternion to rotation matrix
Just copy it to an `Eigen::Quaternion` and use its `matrix()` or `toRotationMatrix()` method.
```cpp
geometry_msgs::Quaternion q_msg;
Eigen::Quaternionf rotation;
rotation.w() = q_msg.w;
rotation.x() = q_msg.x;
rotation.y() = q_msg.y;
rotation.z() = q_msgs.z;
Eigen::Matrix3f rot_mat = rotation.toRotationMatrix();
```

## Get roll, pitch, yaw from a rotation matrix
This is useful for `pcl::CropBox`, which allows you to specify a rotation for the box, but requires you to specifiy it in roll, pitch, and yaw angles.

```cpp
Eigen::Matrix3f rot_mat;
float roll = atan2(rot_mat(2, 1), rot_mat(2, 2));
float pitch = asin(-rot_mat(2, 0));
float yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));
```
