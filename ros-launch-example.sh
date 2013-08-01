#!/bin/bash
mkdir -p /dev/shm/rosLogs
echo "Sourcing env vars" 2>&1 >> /dev/shm/rosLogs/daemon.log
source /home/nwertzberger/.bashrc 2>&1 >> /dev/shm/rosLogs/daemon.log
echo "Starting ROS as Daemon" 2>&1 >> /dev/shm/rosLogs/daemon.log
roslaunch ros_irobot_controls controls.launch 2>&1 >> /dev/shm/rosLogs/daemon.log
echo "I guess that's it"  2>&1 >> /dev/shm/rosLogs/daemon.log

