marti\_common ![CI](https://github.com/swri-robotics/marti_common/workflows/CI/badge.svg) ![CI](https://github.com/swri-robotics/marti_common/workflows/CI/badge.svg?branch=dashing-devel)
=============

This repository provides various utility packages created at [Southwest Reseach Institute](http://www.swri.org)'s [Intelligent Vehicle Systems](http://www.swri.org/4org/d10/isd/ivs/default.htm) section for working with [Robot Operating System(ROS)](http://www.ros.org).  This branch adds support for ROS 2 Dashing.  Most packages from ROS 1 have been ported, but a few have been removed due to being unnecessary or redundant, and some functionality is not implemented yet.

Overview
--------

Installation (ROS Kinetic, Melodic, Noetic)
-------------

You can install any of the packages in this repository with apt-get:

    sudo apt-get install ros-${ROS_DISTRO}-<package>

Building From Source (ROS Kinetic, Melodic, Noetic)
------------

These directions assume you have already set up rosdep. See the [rosdep documentation](http://wiki.ros.org/rosdep) on the ROS wiki for help setting up rosdep.

1. If you don't have a colcon workspace, create one:

    ```bash
    mkdir $HOME/workspace/src
    cd $HOME/workspace/src
    ```

2. Check out the source code

    ```bash
    cd $HOME/workspace/src
    git clone https://github.com/swri-robotics/marti_common.git
    ```

3. Install dependencies:

    ```bash
    # (In the root of this repository)
    rosdep install --from-paths . --ignore-src
    ```

4. Build

    ```bash
    cd $HOME/workspace
    colcon build
    ```
