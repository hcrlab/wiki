# How to create reusable Python modules

The `src` folder of a ROS package should contain both Python and C++ code. C++ code goes directly in `src`, while Python code goes in `src/<package_name>`. This is because `<package_name>` will be the name of the Python module.

Say you want `pr2_pick_manipulation` to be a module that people can reuse in other packages.

- `mkdir -p pr2_pick_manipulation/src/pr2_pick_manipulation`
- `vim pr2_pick_manipulation/src/pr2_pick_manipulation/grasp_generation.py`
- `vim pr2_pick_manipulation/src/pr2_pick_manipulation/__init__.py`
  - Inside `__init__.py`, import names inside the `pr2_pick_manipulation` folder like `from .grasp_generation import GraspGenerator`
- Create a setup.py in the root of the package directory:
```py
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['pr2_pick_manipulation'],
    package_dir={'': 'src'})

setup(**setup_args)
```

- Uncomment catkin_python_setup() in the CMakeLists.txt

Now, outside of the package, you can depend on `pr2_pick_manipulation` the same way you depend on any other package. You should be able to import code from your module like so:

```
from pr2_pick_manipulation import GraspingStrategy
```
