# Unit testing in ROS
It's possible to write unit tests and slightly more complicated ROS node tests using some frameworks.
- C++: [gtest ROS wiki](http://wiki.ros.org/gtest), [gtest official documentation](https://code.google.com/p/googletest/wiki/V1_7_Primer)
- Python: [unittest ROS wiki](http://wiki.ros.org/unittest), [unittest official documentation](https://docs.python.org/2/library/unittest.html)
- Other ROS resources: [rostest](http://wiki.ros.org/rostest), [UnitTesting](http://wiki.ros.org/UnitTesting)

## C++ info

### Unit testing
If you have purely algorithmic code that doesn't depend on ROS, unit testing is easy. ROS has built-in support for the googletest (gtest) C++ library.

#### Write your unit test
- Create a folder called `test/` in your project directory.
- Create a file called `test/myproject_test.cpp`

Template:
```cpp
#include "myproject/linked_list.h"

#include <gtest/gtest.h>

namespace myproject {
TEST(TestLinkedList, IsEmptyInitially) {
  LinkedList list;
  EXPECT_EQ(0, list.Size());
}

TEST(TestLinkedList, CanAddElement) {
  LinkedList list;
  list.Add(5);
  EXPECT_EQ(1, list.Size());
}

// More tests 
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

#### Add your test to CMakeLists
Add this to the bottom of your CMakeLists.txt:
```cmake
catkin_add_gtest(${PROJECT_NAME}-test test/myproject_test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
```

#### Run your test
In the terminal, run:
```bash
cd ~/catkin_ws
catkin_make run_tests_mypackage
```

### Node testing
It's possible to test ROS nodes in isolation. Let's say we want to test a node that listens for messages on a topic, and publishes messages on another topic. To test this, you launch the node under test, as well as another node which publishes fake messages and inspects messages published by the node under test.

#### Write the test
- Create a folder called test/ in your project directory.
- Create a file called test/mynode_test.cpp

This version of the test uses a gtest test fixture.

It takes time to for the publisher/subscribers to connect, as well as to publish/receive messages. So, we introduce some additional methods which block our unit test until the node is ready.

Template:
```cpp
#include "myproject/mynode.h"

#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

// TODO: add other includes as needed

namespace myproject {
class MyNodeTest : public ::testing::Test {
 public:
  MyNodeTest()
      : node_handle_(),
        publisher_(
            node_handle_.advertise<Feedback>(
                "/input_topic", 5)),
        subscriber_(
            node_handle_.subscribe("/output_topic", 5,
                                   &MyNodeTest::Callback,
                                   this)) {
  }

  /*
   * This is necessary because it takes a while for the node under
   * test to start up.
   */
  void SetUp() {
    while (!IsNodeReady()) {
      ros::spinOnce();
    }
  }

  void Publish(int num /* TODO: add necessary fields */) {
    MyInputMessage message;
    // TODO: construct message.
    publisher_.publish(message);
  }

  /*
   * This is necessary because it takes time for messages from the
   * node under test to reach this node.
   */  
  boost::shared_ptr<const MyOutputMessage> WaitForMessage() {
    // The second parameter is a timeout duration.
    return ros::topic::waitForMessage<MyOutputMessage>(
        subscriber_.getTopic(), ros::Duration(1));
  }

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;

  /*
   * This callback is a no-op because we get the messages from the
   * node under test using WaitForMessage().
   */
  void Callback(const MyOutputMessage& event) {
  }

  /*
   * See SetUp method.
   */
  bool IsNodeReady() {
    return (publisher_.getNumSubscribers() > 0)
        && (subscriber_.getNumPublishers() > 0);
  }
};

TEST_F(MyNodeTest, NonZeroInputDoesSomething) {
  Publish(1);
  auto output = WaitForEvent();
  ASSERT_TRUE(output != NULL);
  EXPECT_EQ(10, output->some_output);
}

TEST_F(MyNodeTest, ZeroInputDoesNothing) {
  Publish(0);
  auto output = WaitForEvent();
  ASSERT_TRUE(output == NULL);
}
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mynode_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
```

#### Set up the build
Create a launch file in test/ called my_node_test.test:
```xml
<launch>
  <node pkg="mypackage" type="mynode" name="mynode" />
  <test test-name="mynode_test" pkg="mypackage" type="mynode_test" />
</launch>
```

In your CMakeLists.txt, add rostest to find_package, e.g.:
```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostest
  message_generation
)
```

Add this to the bottom of your CMakeLists.txt:
```cmake
add_rostest_gtest(click_transformer_test
  test/click_transformer.test
  test/click_transformer_test.cpp
)
target_link_libraries(click_transformer_test ${catkin_LIBRARIES})
```

#### Run the test
```bash
cd ~/catkin_ws
catkin_make
catkin_make run_tests_mypackage
```
or
```bash
rostest mypackage my_node_test.test
```

## Python info
Testing Python should be similar, it just uses a different framework. See the resources at the top for more info. Then add details here!

## Test Types
Typical tests can be divided into the following categories:
* **Node tests** - single node tests
    * **Continuous diagnostics** - continuously published diagnostic messages informing about the state of the node
    * **Internal tests** - tests performed internally by the node itself, useful to test detailed internal behaviors
    * **External tests** - tests analyzing the outputs produced by the node for certain given inputs
* **System tests** - tests involving multiple interacting nodes 

Please see below for recommendations regarding implementation for each category.

### Continuous Node Diagnostics
This type of tests should be implemented using the [diagnostics ROS system](http://wiki.ros.org/diagnostics) and more specifically the [diagnostic_updater API](http://wiki.ros.org/diagnostic_updater).

In short, the API provides a convenient way of reporting any kind of diagnostic information at regular intervals. The node can define multiple diagnostic tests which will be periodically invoked in order to gather the relevant diagnostic information and automatically send it out to `/diagnostics` topic. The `diagnostic_aggregator` package can categorize and analyze diagnostics at runtime which is then displayed conveniently in `rqt` either as in the form of a tree or using nice visual robot dashboards.

### Internal Node Tests
This type of tests should be implemented using the [diagnostics ROS system](http://wiki.ros.org/diagnostics) and more specifically the [self_test API](http://wiki.ros.org/self_test). Those tests can be invoked using a service call, but also from a `*.test` file using `selftest_rostest` in [self_test package](http://wiki.ros.org/self_test).

In short, the API provides a convenient way of defining "test services" which when invoked run some internal test process within the node. Those services are very easy to define using the API and can then be invoked at any point when the node is running. The `selftest_rostest` script can then be used to invoke those tests from the same `*.test` files that implement system and external node tests described below.

### External Node Tests
Those tests should be implemented using the [rostest ROS system](http://wiki.ros.org/rostest). Some sparse general info on writing such tests is provided [here](http://wiki.ros.org/rostest/Writing). They generally consist of:
* a `*.test` file in the `test` sub-directory of a package, which starts the:
    * robot drivers or simulation
    * node that is tested
    * nodes/scripts providing input data or action/service calls
    * a unit test node evaluating the outputs
* unit test scripts, also in the `test` sub-directory of a package

It is important to note the following:
* `rostest` runs each test using a dedicated rosmaster that is operating on a different port. As a result, it is not possible to connect to it using nodes or e.g. `rostopic` unless the ROSMASTER variable is modified to represent that special port.
* Each test is run using a new clean rosmaster and all the nodes started for the previous test are killed. This is the case even if the tests are defined in the same `*.test` file. There can be no dependencies between the tests.

### System Tests

Those tests should be implemented in the same way as above, using the [rostest ROS system](http://wiki.ros.org/rostest). 
