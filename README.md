# UAV FSM Generation Model & Simulation
### MSc. in Logic, Computation and Artificial Intelligence
### Computational Methods In Artificial Life
Authors: David Tejero Ruiz & Miguel Gil Castilla

## About
This project aims to model the interacton of UAVs by using artificial forces. This architecture allows to generate emergent swarm behaviours from simpler individual rules. Automatic formation of regular polyhedron, obstable avoidance, follow-the-leader algorithm implementation, etc.

This repository contains a python script to simulate the proposed forces testing different parameter settings in various scenarios.

This model has been tested in a simulation environment using Gazebo & ROS melodic and noetic & Ardupilot based on the tools provided by [Intelligent-Quads](https://github.com/Intelligent-Quads) and the Ardupilot's SITL model.

The used repositories have been forked and modified in order to implement a realistic 4-UAV swarm simulation using our force model.
- [Ardupilot's SITL v4.3.1](https://github.com/davidtr99/ardupilot/tree/Copter-4.3.1): Adapted to 4-UAV swarm instances
- [IQ_SIM](https://github.com/davidtr99/iq_sim): Simulation Tools (only changed spawn angles in the file multi-drone.world)
- [IQ_GNC](https://github.com/davidtr99/iq_gnc): Guidance Navigation Control using our forces model.


\todo images

# Installation

## Ardupilot

### Clone ArduPilot

In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/davidtr99/ardupilot/tree/Copter-4.3.1
cd ardupilot
git checkout Copter-4.3.1
git submodule update --init --recursive
```

### Install dependencies:
```
sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect
```

### Use pip (Python package installer) to install mavproxy:
```
sudo pip install future pymavlink MAVProxy
```

MAVProxy is a fully-functioning GCS for UAV’s. The intent is for a minimalist, portable and extendable GCS for any UAV supporting the MAVLink protocol (such as one using ArduPilot). For more information check out http://ardupilot.github.io/MAVProxy/html/index.html

Open `~/.bashrc` for editing:
```
gedit ~/.bashrc
```

Add these lines to end of `~/.bashrc` (the file open in the text editor):
```
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=/usr/lib/ccache:$PATH
```

Save and close the text editor.

Reload `~/.bashrc`:
```
. ~/.bashrc
```

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

## Setup your catkin workspace

 We use `catkin build` instead of `catkin_make`. Please install the following:
```
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools
```

Then, initialize the catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

## IQ_SIM

### Install dependencies:

Install the following ROS packages:
```
sudo apt install ros-<rosdistro>-mavros

sudo apt install ros-<rosdistro>-mavros-extras

```
or do it  from source:
```
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables
```
source ~/.bashrc
```

install geographiclib dependancy 
```
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```


Clone our fork:
```
cd ~/catkin_ws/src
git clone https://github.com/davidtr99/iq_sim
```
Our repository should now be copied to `~/catkin_ws/src/iq_sim/` (don't run this line. This is just saying that if you browse in the file manager, you will see those folders).

run the following to tell gazebo where to look for the iq models 
```
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/iq_sim/models" >> ~/.bashrc
```
Inside `catkin_ws`, run `catkin build`:

```
cd ~/catkin_ws
catkin build
```
update global variables
```
source ~/.bashrc
```

## IQ_GNC
Simply clone our fork:
```
git clone https://github.com/davidtr99/iq_gnc
```

and build the workspace:
```
cd ~/catkin_ws/
catkin build
```

# Tutorial

To demonstrate the functionalities you can try this:

At first go to your workspace and charge the enviroment variables:
```
cd ~/catkin_ws/
source devel/setup.bash
```

Then run the simulator:
```
roslaunch iq_sim multi_drone.launch
```

Open four new terminals and do the following:

```
# in all of them:
cd ~/ardupilot/ArduCopter/

# first terminal:
sim_vehicle.py -v ArduCopter -f gazebo-drone1 -I0
# second:
sim_vehicle.py -v ArduCopter -f gazebo-drone2 -I1
# third:
sim_vehicle.py -v ArduCopter -f gazebo-drone3 -I2
# forth:
sim_vehicle.py -v ArduCopter -f gazebo-drone4 -I3

```

The last commands will create four instances of the ardupilot SITL, which you will connect later to the simulation in gazebo.

Until you can see that each one is using the GPS, please do not continue with the tutorial.

...

All of them done? You are a beast, ok so we can keep going.

Open a new terminal (yes, another one we have two weeks to do this what do you want?) and enable the mavros instances:

```
roslaunch iq_sim multi-apm.launch
```

We have all the necessary tools to begin our experiments. 

You can try these last commands to watch the algorithim working:

```
#new terminal

cd ~/catkin_ws/

roslaunch iq_gnc multi_spawn.launch
```

You will see the drones taking off and when they are holding a position in a concrete height, introduce the last command:

```
# last terminal, I promise
cd ~/catkin_ws/

rosrun iq_gnc fsm_dev.py

```

Now you

