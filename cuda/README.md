# Installation on Ubuntu 12.04

These directions are taken from the [CUDA manual](http://developer.download.nvidia.com/compute/cuda/7_0/Prod/doc/CUDA_Getting_Started_Linux.pdf)
 and worked on the computer `walle`. 
 
In my experience, updating/installing NVidia drivers can be a perilous procedure.

**Take special note of page 6 of the
 [CUDA manual](http://developer.download.nvidia.com/compute/cuda/7_0/Prod/doc/CUDA_Getting_Started_Linux.pdf), which 
 describes how to deal with existing NVidia drivers or CUDA installations.**

1. Download and install the `.deb` file from [here](https://developer.nvidia.com/cuda-downloads?sid=913002).
2. `sudo apt-get update`
3. `sudo apt-get install cuda`
4. Reboot.
5. `export PATH=/usr/local/cuda-7.0/bin:$PATH` and `export LD_LIBRARY_PATH=/usr/local/cuda-7.0/lib64:$LD_LIBRARY_PATH`.

and finally, to test it out:

6. `cuda-install-samples-7.0.sh <dir>` (`<dir>` can just be `$HOME`).
7. `cd <dir> && make`
