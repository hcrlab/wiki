# Code coverage
Code coverage is a measure of how much of your program is tested by your tests.
Typically, this measure is the number of lines of code run by your tests divided by the total number of lines of code, excluding useless stuff like comments and boilerplate.

A good target for code coverage is 80% or more.
However, having high test coverage doesn't necessarily mean your tests are useful, it just means your tests lead to every line of code being executed.
Additionally, having low test coverage doesn't necessarily mean your tests aren't comprehensive enough.
You may have lots of tests for complex parts of your code and no tests for very simple parts of your code.
Nonetheless, code coverage is still a useful metric for knowing about how well-tested a project is.

## Computing code coverage for C++ projects
`gcov` is a tool for measure code coverage for C++.
Most systems will already have `gcov` installed, since it comes with gcc.
To use it, you first need to compile your code with the options `-g -O0 --coverage`:

In CMakeLists.txt, that would look like:
```cmake
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage") # Enable code coverage
  catkin_add_gtest(my_test test/my_test.cpp)
  target_link_libraries(my_test
    my_library
    ${catkin_LIBRARIES}
  )
endif()
```

Next, you need to actually run your tests.
This generates extra files in the catkin `build` directory, which contain the coverage information.

### Extracting and filtering coverage information
`lcov` is a graphical frontend for `gcov`.
Install `lcov` with `sudo apt-get instal lcov` if you don't have it.

The following will extract the coverage data from the `build` directory and saves it to `coverage.info`
```bash
cd ~/catkin_ws/build
lcov --directory . --capture --output-file coverage.info
lcov --list coverage.info
```

When you run `gcov`, it will include system files and core ROS code in the code coverage count.
Since you are probably only interested in the coverage for code you wrote, you need to filter to the coverage data.
Filter the data using `lcov --remove`.
In this example, we filter all ROS code from `/opt/*`, system code from `/usr/*`, generated code from `/devel/*`, and code from the tests themselves (`*test_*` and `*_test*`):
```bash
lcov --remove coverage.info '/opt/*' '/usr/*' '*/devel/*' '*test_*' '*_test*' --output-file coverage.info
lcov --list coverage.info
```

## Uploading and viewing coverage on Coveralls
You can use `lcov` to view the coverage information visually.
However, you can use [Coveralls](https://coveralls.io/) to integrate code coverage checking with Github and Travis.

Coveralls offers a few nice features:
- Cloud hosting of coverage information
- See history of code coverage across all pushes / pull requests
- Prevent pull requests from being merged if coverage is too low or falls too much
- Auto-updating badge showing code coverage, which you can add to your README.md
  - Unfortunately, it seems like the badge only updates when you push to a repository, and not when you merge a pull request.

If you have [Travis set up for continuous integration](continuous_integration.md), then you can automatically upload coverage information to Coveralls for every push and pull request.
First, add this to the install script in .travis.yml: `gem install coveralls-lcov`.
Then, after extracting and filtering your `coverage.info`, simply run `coveralls-lcov coverage.info`

- See [rapid's .travis.yml](https://github.com/jstnhuang/rapid/blob/master/.travis.yml) for an example.

## Computing code coverage for Python projects
See the [PR2/pr2_pbd](https://github.com/PR2/pr2_pbd/search?utf8=%E2%9C%93&q=coverage) package for an example.
