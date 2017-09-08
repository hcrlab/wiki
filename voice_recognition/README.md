# Voice recognition

## Installation
The simplest way to do voice recognition in ROS is to use [pocketsphinx](http://wiki.ros.org/pocketsphinx):
```
sudo apt-get install ros-VERSION-pocketsphinx
sudo apt-get install ros-indigo-pocketsphinx
```

## Creating language model files
pocketsphinx requires certain language model files, which are automatically generated using a tool called `lmtool`.
The simplest way to generate these files is to use an online version of lmtool, hosted online by CMU.

First, create a "corpus" file, a plain text file that contains all the commands you would like the robot to recognize, one command per line.
If you put spaces in between words, pocketsphinx will be able to recognize each word in the vocabulary and recognize sentences that aren't in the corpus file.
One trick we have done in the past is to put dashes in between words for each command, so that each command is a unique "word", like so:
```
create-new-action
save-pose
record-tabletop-objects
```

Next, go to the [CMU lmtool website](http://www.speech.cs.cmu.edu/tools/lmtool-new.html) and upload your corpus file.
It will generate the appropriate language model files and provide you with a link to download them.

## Manual installation of pocketsphinx
If you want to manually install pocketsphinx, look at these links:
- pocketsphinx: http://cmusphinx.sourceforge.net/wiki/tutorialpocketsphinx#installation and http://cmusphinx.sourceforge.net/wiki/download
- sphinxbase: http://cmusphinx.sourceforge.net/wiki/download
