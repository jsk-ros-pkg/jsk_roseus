# jsk_roseus

[![Build Status](https://travis-ci.org/jsk-ros-pkg/jsk_roseus.png?branch=master)](https://travis-ci.org/jsk-ros-pkg/jsk_roseus)
[![Documentation Status](https://readthedocs.org/projects/euslisp-docs/badge/?version=latest)](http://euslisp-docs.readthedocs.org/en/latest/roseus/)

## Tips

### Run roseus on gdb
```
gdb --args bash roseus foo.l
```


### Use roseus with euslisp built from source

To use euslisp built from source, we need to create upstream workspace and then overlay it to your workspace.

1. Create the upstream workspace

    Assumes you already installed `ros-<your distro>-desktop-full`.

    ```bash
    source /opt/ros/<your distro>/setup.bash
    mkdir -p ~/ros/$ROS_DISTRO_parent/src/
    cd ~/ros/$ROS_DISTRO_parent/src
    wstool init
    wget https://raw.githubusercontent.com/jsk-ros-pkg/jsk_roseus/master/setup_upstream.sh -O /tmp/setup_upstream.sh
    bash /tmp/setup_upstream.sh -w ..
    cd ~/ros/$ROS_DISTRO_parent/
    catkin init
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # This is optional
    catkin build
    source ~/ros/$ROS_DISTRO_parent/devel/setup.bash
    ```
    
    You can check if `euslisp` built from source is available by running `which irteusgl`.

2. Build downstream packages using euslisp built from source

    Configure your catkin workspace to overlay the upstream workspace and build `roseus`.
    
    ```bash
    mkdir -p ~/ros/$ROS_DISTRO/src  # If you not yet create your workspace
    cd ~/ros/$ROS_DISTRO/src
    wstool init  # If you don't yet create your workspace
    wstool set jsk-ros-pkg/jsk_roseus --git https://github.com/jsk-ros-pkg/jsk_roseus.git -v master -u -y
    rosdep install --from-paths . -i -r -n -y  # By running this, all dependencies will be installed
    cd ~/ros/$ROS_DISTRO
    catkin init  # If you don't yet create your workspace
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # This is optional
    catkin build roseus
    source ~/ros/$ROS_DISTRO/devel/setup.bash
    ```

## Deb Status

| Package | Indigo (Saucy) | Indigo (Trusty) | Jade (Trusty) | Jade (Vivid) | Kinetic (Wily) | Kinetic (Xenial) |
|--------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| euslisp (32-bit) | [![Build Status](http://build.ros.org/job/Ibin_uS32__euslisp__ubuntu_saucy_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uS32__euslisp__ubuntu_saucy_i386__binary/) | [![Build Status](http://build.ros.org/job/Ibin_uT32__euslisp__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uT32__euslisp__ubuntu_trusty_i386__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uT32__euslisp__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uT32__euslisp__ubuntu_trusty_i386__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uV32__euslisp__ubuntu_vivid_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uV32__euslisp__ubuntu_vivid_i386__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uW32__euslisp__ubuntu_wily_i386__binary/badge/icon)](http://build.ros.org/job/Kbin_uW32__euslisp__ubuntu_wily_i386__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uX32__euslisp__ubuntu_xenial_i386__binary/badge/icon)](http://build.ros.org/job/Kbin_uX32__euslisp__ubuntu_xenial_i386__binary/) |
| euslisp (64-bit) | [![Build Status](http://build.ros.org/job/Ibin_uS64__euslisp__ubuntu_saucy_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uS64__euslisp__ubuntu_saucy_amd64__binary/) | [![Build Status](http://build.ros.org/job/Ibin_uT64__euslisp__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uT64__euslisp__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uT64__euslisp__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uT64__euslisp__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uV64__euslisp__ubuntu_vivid_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uV64__euslisp__ubuntu_vivid_amd64__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uW64__euslisp__ubuntu_wily_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uW64__euslisp__ubuntu_wily_amd64__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uX64__euslisp__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__euslisp__ubuntu_xenial_amd64__binary/) |
| jskeus (32-bit) | [![Build Status](http://build.ros.org/job/Ibin_uS32__jskeus__ubuntu_saucy_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uS32__jskeus__ubuntu_saucy_i386__binary/) | [![Build Status](http://build.ros.org/job/Ibin_uT32__jskeus__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uT32__jskeus__ubuntu_trusty_i386__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uT32__jskeus__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uT32__jskeus__ubuntu_trusty_i386__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uV32__jskeus__ubuntu_vivid_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uV32__jskeus__ubuntu_vivid_i386__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uW32__jskeus__ubuntu_wily_i386__binary/badge/icon)](http://build.ros.org/job/Kbin_uW32__jskeus__ubuntu_wily_i386__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uX32__jskeus__ubuntu_xenial_i386__binary/badge/icon)](http://build.ros.org/job/Kbin_uX32__jskeus__ubuntu_xenial_i386__binary/) |
| jskeus (64-bit) | [![Build Status](http://build.ros.org/job/Ibin_uS64__jskeus__ubuntu_saucy_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uS64__jskeus__ubuntu_saucy_amd64__binary/) | [![Build Status](http://build.ros.org/job/Ibin_uT64__jskeus__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uT64__jskeus__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uT64__jskeus__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uT64__jskeus__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uV64__jskeus__ubuntu_vivid_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uV64__jskeus__ubuntu_vivid_amd64__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uW64__jskeus__ubuntu_wily_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uW64__jskeus__ubuntu_wily_amd64__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uX64__jskeus__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__jskeus__ubuntu_xenial_amd64__binary/) |
| jsk_roseus (32-bit) | [![Build Status](http://build.ros.org/job/Ibin_uS32__jsk_roseus__ubuntu_saucy_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uS32__jsk_roseus__ubuntu_saucy_i386__binary/) | [![Build Status](http://build.ros.org/job/Ibin_uT32__jsk_roseus__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Ibin_uT32__jsk_roseus__ubuntu_trusty_i386__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uT32__jsk_roseus__ubuntu_trusty_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uT32__jsk_roseus__ubuntu_trusty_i386__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uV32__jsk_roseus__ubuntu_vivid_i386__binary/badge/icon)](http://build.ros.org/job/Jbin_uV32__jsk_roseus__ubuntu_vivid_i386__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uW32__jsk_roseus__ubuntu_wily_i386__binary/badge/icon)](http://build.ros.org/job/Kbin_uW32__jsk_roseus__ubuntu_wily_i386__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uX32__jsk_roseus__ubuntu_xenial_i386__binary/badge/icon)](http://build.ros.org/job/Kbin_uX32__jsk_roseus__ubuntu_xenial_i386__binary/) |
| jsk_roseus (64-bit) | [![Build Status](http://build.ros.org/job/Ibin_uS64__jsk_roseus__ubuntu_saucy_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uS64__jsk_roseus__ubuntu_saucy_amd64__binary/) | [![Build Status](http://build.ros.org/job/Ibin_uT64__jsk_roseus__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Ibin_uT64__jsk_roseus__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uT64__jsk_roseus__ubuntu_trusty_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uT64__jsk_roseus__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/job/Jbin_uV64__jsk_roseus__ubuntu_vivid_amd64__binary/badge/icon)](http://build.ros.org/job/Jbin_uV64__jsk_roseus__ubuntu_vivid_amd64__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uW64__jsk_roseus__ubuntu_wily_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uW64__jsk_roseus__ubuntu_wily_amd64__binary/) | [![Build Status](http://build.ros.org/job/Kbin_uX64__jsk_roseus__ubuntu_xenial_amd64__binary/badge/icon)](http://build.ros.org/job/Kbin_uX64__jsk_roseus__ubuntu_xenial_amd64__binary/) |
