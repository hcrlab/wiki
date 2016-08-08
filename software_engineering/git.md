# Git resources

- [HCR Lab Git tutorial (slides)](https://docs.google.com/presentation/d/11z_sScRlFVTSX5wHhAuJy4rJuzPUAVG-oyEAZRCHdmQ/pub?start=false&loop=false)
- [HCR Lab Git tutorial (document)](https://docs.google.com/document/d/1lLyPA6oByAGe0bH3h3wdsV8cfy5LskyuxiVGNSr4wos/pub)
- [Git cheat sheet](http://www.git-tower.com/blog/git-cheat-sheet-detail/)
- [Git learning game](pcottle.github.io/learnGitBranching/?NODEMO)

## Adding your workstation as a remote
A common task is that you want to test some changes on the robot, while developing locally on your workstation.
One way to synchronize your work from your computer to the robot is to push and pull from Github, but this can be annoying if you are still in a testing phase where your code is full of bugs.
Instead, you can add your workstation as a remote on the robot, and you can pull changes directly from your workstation to the robot, instead of using Github as an intermediary.

Assume you are working on a repo called `my_repo` on a branch named `my_experiment`.
On the robot, go to the root of your repository and enter:
```
git remote add silvertuna silvertuna:/home/gbluefin/catkin_ws/src/my_repo
git remote update
git checkout -b my_experiment
git pull silvertuna my_experiment
```
