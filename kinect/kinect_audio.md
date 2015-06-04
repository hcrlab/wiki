# Kinect audio

The XBox 360 Kinect has 4 microphones in it, but there is no ROS interface for it. However, open-source drivers do exist for it, via libfreenect. This page explains how to install the software necessary to use the Kinect's microphone. We have already done this with Rosie.

## Installing libusb
Ubuntu 12.04 has an outdated version of libusb, which doesn't work with libfreenect. It also doesn't work to install libusb-1.0.0 via apt-get. Instead, we compile it from source.
```bash
mkdir ~/local
cd ~/local
git clone git@github.com:libusb/libusb.git
cd libusb
./autogen.sh
make
sudo make install
```

This installs the newer version of libusb, with the header in `/usr/local/include/libusb/libusb-1.0.h`, and the compiled libraries in `/usr/local/lib/libusb-1.0.*`

## Installing libfreenect with audio support
The default version of libfreenect does not come with audio support, so we must again compile it from source:
```bash
cd ~/local
git clone git@github.com:OpenKinect/libfreenect.git
cd libfreenect
```

By default, CMake will look for libusb in `/usr/include` and `/usr/lib`, where the old version of libusb is. So we need to make it look for the new version we just installed to `/usr/local`:
```bash
vim cmake_modules/Findlibusb-1.0.cmake
```

Change the ordering of `/usr/include` and `/usr/local/include`, so that `/usr/local/include` is above `/usr/include`. Do the same for `/usr/lib` and `/usr/local/lib`, we want `/usr/local/lib` to come first.

Now we can generate the Makefile:
```bash
cd libfreenect
mkdir build
cd build
cmake -L .. -DBUILD_REDIST_PACKAGE=OFF
```

`-DBUILD_REDIST_PACKAGE=OFF` is necessary for audio support.

Confirm that the output says:
```
-- Found libusb-1.0:
--  - Includes: /usr/local/include/libusb-1.0
--  - Libraries: /usr/local/lib/libusb-1.0.so
```

Now build libfreenect.
```bash
make
sudo make install
sudo ldconfig /usr/local/lib
```

To get WAV recording working, you first need to run the example binary from within the build folder. This will upload firmware to the Kinect, then start recording.
```bash
cd build
bin/freenect-wavrecord
```

From then on, you should be able to use the freenect-wavrecord command from anywhere. Try recording something:
```bash
freenect-wavrecord
```
