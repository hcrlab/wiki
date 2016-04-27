# PCL C++ tips

## Designing APIs that work with point clouds
### Should my algorithm take in a PointCloud by value, reference, or pointer?

By pointer.
In particular, a `PointCloud::Ptr`, which is a typedef for a `boost::shared_ptr<PointCloud>`.

#### Why not by value?
Taking in a point cloud by value is bad because it will make a copy of the point cloud:
```cpp
bool DoSomething(PointCloud<PointXYZRGB> cloud); // Bad, makes a copy of the point cloud given to it when called.
```

#### Why not by reference?
Taking a point cloud by reference is better, but it is less flexible.
Although some PCL algorithms take in point clouds by reference, in most cases, PCL algorithms require a pointer:
```cpp
bool DoSomething(const PointCloud<PointXYZRGB>& cloud) {
  pcl::CropBox cb;
  cb.setInputCloud(...); // Can't pass in cloud here, needs a PointCloud::Ptr
}
```

If you have a `PointCloud` and you need `PointCloud::Ptr`, one option is to call `makeShared()`.
However, this is undesirable because `makeShared()` will make a copy of the point cloud on the heap, which it then returns a pointer to.
```cpp
bool DoSomething(const PointCloud<PointXYZRGB>& cloud) {
  pcl::CropBox cb;
  cb.setInputCloud(cloud.makeShared()); // Bad, makes a copy of the point cloud
}
```

#### Why by PointCloud::Ptr?
A `PointCloud::Ptr` is the best option, because it works with all algorithms, whether they use require pointer or reference arguments:
```cpp
bool NeedsRef(const PointCloud<PointXYZRGB>& cloud);
void NeedsPtr(PointCloud<PointXYZRGB>::Ptr cloud);

bool DoSomething(PointCloud<PointXYZRGB>::Ptr cloud) {
  NeedsRef(*cloud);
  NeedsPtr(cloud);
}
```

#### Exception to the rule
One exception to this rule is that if you are sure that you aren't going to call any methods that need a pointer, and you are not going to modify the point cloud, then it's best to take the pointer in as a const reference:
```cpp
int CountSomething(const PointCloud<PointXYZRGB>& cloud);
```

Taking in the cloud as a const reference is a signal to other programmers that the function will have read-only access to the cloud, whereas a function that takes in a pointer can always modify the cloud.
It is possible to pass in a `PointCloud::ConstPtr` to enforce the read-only constraint, but the convention in the C++ community is to use const references to represent read-only objects.

### How do I represent a subset of a point cloud?

Use `PointIndices`.
For the same reason as above, it's better to use `PointIndices::Ptr` throughout rather than `PointIndices` because it's more flexible.

A common task is to filter point clouds in some way.
For example, removing a table plane or segmenting objects.
It may be tempting to create a new point cloud to hold the filtered output, but this involves copying all the point cloud data in that subset.
Instead, you can simply track the indices of the original point cloud that are in the subset using `PointIndices`.
`PointIndices` is just a wrapper around a `vector<int>`.
Most PCL algorithms have a method called `setIndices` in addition to `setInputCloud`, which runs the algorithm on just the subset of the cloud specified by the indices.

The example below is wrong, it copies point cloud data unnecessarily into `points_above_table`:
```cpp
void FindObjects(PointCloud<PointXYZ>::Ptr cloud) {
  Table table = FindTable(cloud);
  PointCloud<PointXYZ>::Ptr points_above_table(new PointCloud<PointXYZ>);
  // Add points in cloud above table to points_above_table
  vector<Object> objects = FindObjects(points_above_table);
}
```

The example below is better, it uses `PointIndices` instead.
This is the minimum amount of data needed to represent the subset.
```cpp
void FindObjects(PointCloud<PointXYZ>::Ptr cloud) {
  Table table = FindTable(cloud);
  PointIndices::Ptr indices_above_table(new PointIndices);
  // Add points in cloud above table to indices_above_table
  vector<Object> objects = FindObjects(cloud, indices_above_table);
  // FindObjects uses setInputCloud(cloud) and setIndices(indices_above_table)
}
```

As a result of this, most of your algorithms should accept both a `PointCloud::Ptr` and a `PointIndices::Ptr` as input.

If you need to get a `PointCloud` from a `PointCloud::Ptr` and some `PointIndices`, you can always use [`pcl::ExtractIndices`](http://docs.pointclouds.org/1.7.0/classpcl_1_1_extract_indices.html).
