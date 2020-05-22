# UBLOX_read

A library for parsing UBLOX packets and interfacing with UBLOX gps receivers. It has been designed for use with the M8N and F9P GNSS receivers.

This library provides rather basic functionality and is designed to work under a linux environment, however it should probably work in Windows or Mac as it uses the cross-platform [`async_comm`](https://github.com/dpkoch/async_comm) library as the serial interface.

The UBX parsing functionality is abstracted into a library for easy integration in other projects.  Example usage is given in the `main.cpp` file.

Documentation found [here](https://magiccvs.byu.edu/wiki/#!index.md)

# TL;DR 

## Option 1: One GNSS Module
`roslaunch ublox standard.launch`

## Option 2: One Base, One Rover, One Computer
`roslaunch ublox OneComp.launch`

## Option 3: One Base, One Rover, Two Computers
Modify IP addresses of `base_host` and `rover_host` in `base.launch` and `rover.launch`
On Computer #1: `roslaunch ublox base.launch`
On Computer #2: `roslaunch ublox rover.launch`

`rostopic echo /base/PosVelTime`: Access base GNSS Data
`rostopic echo /rover/RelPos`: Access RelPos data

