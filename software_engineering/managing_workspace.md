# Managing lots of workspaces with wstool

[`wstool`](http://wiki.ros.org/wstool) is a useful tool for managing workspaces that have lots of Git repositories in them.
If you want to get the latest updates from all of them, normally you need to `cd` into each folder and run `git pull` manually.
`wstool` enables you to do this with a single command: `wstool up`

## Usage
- Initialize the workspace
  ```
  cd catkin_ws
  wstool init src
  ```
  
- Add new or existing repositories
  ```
  cd src
  wstool set pr2_pbd --git git@github.com:PR2/pr2_pbd.git
  wstool set pr2_pbd --git git@github.com:PR2/pr2_pbd.git --version hydro-devel # If not the main branch
  ```
  
- Update all your repos
  ```
  cd src
  wstool up
  wstool -j8 # Run this 8 times faster
  ```

Run `wstool -h` for more usage information.

`wstool` generates a .rosinstall file in the `src` directory.
It is a simple YAML file that you can copy to other machines.
Once copied to another workspace's src directory, you can run `wstool up`.
If a repository doesn't exist yet, it will clone it, otherwise it will update the repository.
