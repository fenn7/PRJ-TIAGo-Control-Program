Your system must satisfy the following prerequisites before running this software package:

Must have an installation of Linux, specifically Ubuntu 20.04 LTS, e.g. from https://releases.ubuntu.com/focal/
Must have Singularity, formerly Apptainer, installed as part of the system. For further information please see: https://apptainer.org/docs/admin/main/installation.html
An installation of Git is also needed, as this project depends on some third-party software.

It is also required to install the TIAGo container, from https://nextcloud.nms.kcl.ac.uk/s/BZtWJtiyP57r8YN/download.

Setup Guide: Creating a catkin workspace

Start up a terminal, and enter the following commands in sequence:
    
mkdir -p ~/catkin_ws/src

cd ~catkin_ws/

catkin_make

Enter the command git clone git@github.com:angusleigh/leg_tracker.git -b noetic 

Create a folder named "var5" inside /catkin\_ws/src. Now clone this repository into that folder.


After doing this, you should have two packages under /src, named 'leg_tracker' and 'var5'. Return to your terminal and once again call catkin_make.


Setup Guide: Launching a sample simulationin Rviz and Gazebo}

Start up 3 new, separate terminals, and call singularity run <path-to-your-tiago-container>.sif in each.

Use cd commands to navigate into ~/catkin_ws/src and then type roslocal in all 4 terminals.

Under /src/ there is a folder named '/sample' that is used as a parameter for a map file. I will refer to the path to this (SAMPLE).

Now launch "roslaunch tiago_2dnav_gazebo tiago_navigation.launch public_sim:=true lost:=true map:=(SAMPLE) world:=empty_world to bring up the simulation.
In another terminal, go to ~/catkin_ws/ and type source devel/setup.bash and then roslaunch leg_tracker joint_leg_tracker.launch. You haare now running everything.
