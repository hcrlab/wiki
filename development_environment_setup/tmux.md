# How to use tmux
tmux is a terminal multiplexer: it enables a number of terminals to be created, accessed, and controlled from a single screen. This is useful for when you need to launch a bunch of ros services, each in their own tab.

There are many good resources on how to use tmux online. This guide focuses on how to use tmux to launch several processes at once.

## Installation
```
sudo apt-get install tmux
```

## The basics

To start a session:
```
tmux new -s <session name>
```

You are now in a new session. A session is like having a new terminal window open. tmux uses Ctrl+b as a prefix to all commands (this can be remapped if you like). To send a literal Ctrl+b through tmux, just type Ctrl+b twice.

To create a new shell:
```
Ctrl+b c
```

This creates a new terminal in the current session. Having multiple terminals is analogous to having multiple tabs open in one terminal window. The terminals are numbered in order of their creation, you can see the numbers in the green bar at the bottom.

To switch between terminals:
```
Ctrl+b n or Ctrl+b <number>
```

To kill a terminal:
```
Ctrl+d or exit
```

When you close your last terminal, you will also kill the session. This is like closing the last tab in your terminal window.

You can create multiple tmux sessions and switch between them. While in a tmux session, you can "detach" from it to go back to your main shell.

To detach from a session:
```
Ctrl+b d
```

To reattach to a session:
```
tmux attach-session -t <session name>
```

If you're detached from the session, you can kill a session using:
```
tmux kill-session -t <session name>
```

Sometimes, you'll have a bunch of output going to standard output. To scroll up in tmux using the scroll wheel, you need to hit:
```
Ctrl+b [
```

Then you can scroll. You can also use vim commands to navigate (j/k go up and down, Ctrl+D/Ctrl+U goes half page down/up, etc.) To get out of this mode, press enter.

## Scripts to launch multiple things at once

You can script tmux. Below is a shell script that creates a new session called rr, which runs ssh, roscore, pr2_dashboard, and interactive manipulation backend/frontend, each in their own terminal. The sleep 1 commands are included to give some commands extra time to run.

```bash
tmux start-server
tmux new-session -d -s rr -n ssh
tmux new-window -t rr:1 -n roscore
tmux new-window -t rr:2 -n dashboard
tmux new-window -t rr:3 -n im_backend
tmux new-window -t rr:4 -n im_frontend

tmux send-keys -t rr:0 "ssh c1" C-m
tmux send-keys -t rr:1 "roscore" C-m
sleep 1
tmux send-keys -t rr:2 "realrobot && rosrun rqt_pr2_dashboard rqt_pr2_dashboard" C-m
tmux send-keys -t rr:3 "realrobot && roslaunch pr2_interactive_manipulation_frontend pr2_interactive_manipulation_desktop.launch" C-m
```
