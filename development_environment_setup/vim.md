# Vim tips

## Plugin manager
Install [Vundle](https://github.com/gmarik/Vundle.vim), which can be used to easily install lots of plugins straight from Github. Once Vundle is installed, you just specify the name of the plugin in your .vimrc, then run `vim` and type `:PluginInstall`.

## Install YouCompleteMe
[YouCompleteMe](https://github.com/Valloric/YouCompleteMe) is an auto-completion engine for C++ and Python. Follow the [installation instructions for Ubuntu](https://github.com/Valloric/YouCompleteMe#ubuntu-linux-x64-super-quick-installation).

When you edit a C++ file, you need a `.ycm_extra_conf.py` file in either the current directory or an ancestor directory. This file specifies the compiler flags for your project, so YouCompleteMe knows where all the include files are and how to compile your code. Basically, these should be the same flags you pass to `g++` or whatever `catkin` ultimately uses. Just copy YouCompleteMe's own `.ycm_extra_conf.py` to your project directory, and replace the `flags` list. An example is shown below:

```py
flags = [
'-Wall',
'-Wextra',
'-Wno-unused-parameter',
'-fno-strict-aliasing',
'-std=c++0x',
'-x',
'c++',
'-isystem',
'/usr/include',
'-isystem',
'/usr/local/include',
'-I',
'/opt/ros/hydro/include',
'-I',
'../../devel/include',
'-I',
'pr2_pick_perception/include',
'-I',
'pr2_pick_manipulation/include',
'-I',
'/usr/include/pcl-1.7',
'-I',
'/usr/include/eigen3'
]
```

Note that we need to add `/opt/ros/hydro/include` and other include directories for our project. Also add `devel/include` in your catkin workspace, which is where generated C++ message, service, and action files go.

Add this to your .vimrc so YouCompleteMe doesn't bother your about loading your .ycm_extra_conf.py
```vim
let g:ycm_confirm_extra_conf = 0
```

## Install ROS plugin
[vim-ros](https://github.com/taketwo/vim-ros) is a great plugin for ROS development.
- Syntax highlights msg, srv, and action files
- :A to jump between .cpp and .h files
- :Rosed to rosed files from inside vim
- Autocompletion of message types, even package names like `$(find openni_launch)/lau...`

Add these lines to your .vimrc to trigger autocompletion for ROS files:
```vim
let g:ycm_semantic_triggers = {
\   'roslaunch' : ['="', '$(', '/'],
\   'rosmsg,rossrv,rosaction' : ['re!^'],
\ }
```

## Auto-format your code
Worry less about braces and whitespace, auto-format your code. See [Automatic code formatting](https://github.com/hcrlab/wiki/blob/master/development_environment_setup/auto_code_formatting.md) to learn how.

## 
