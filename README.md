# UAV FSM Generation Model & Simulation
### MSc. in Logic, Computation and Artificial Intelligence
### Computational Methods In Artificial Life
Authors: David Tejero Ruiz & Miguel Gil Castilla

## About
This project aims to model the interacton of UAVs by using artificial forces. This architecture allows to generate emergent swarm behaviours from simpler individual rules. Automatic formation of regular polyhedron, obstable avoidance, follow-the-leader algorithm implementation, etc.

This repository contains a python script to simulate the proposed forces testing different parameter settings in various scenarios.

This model has been tested in a simulation environment using Gazebo & ROS & Ardupilot based on the tools provided by [Intelligent-Quads](https://github.com/Intelligent-Quads) and the Ardupilot's SITL model.

The used repositories have been forked and modified in order to implement a realistic 4-UAV swarm simulation using our force model.
- [IQ_SIM](https://github.com/davidtr99/iq_sim): Simulation Tools
- [IQ_GNC](https://github.com/davidtr99/iq_gnc): Guidance Navigation Control using our forces model.
- [Ardupilot's SITL v4.3.1](https://github.com/davidtr99/ardupilot/tree/Copter-4.3.1): Adapted to 4-UAV swarm instances

\todo images

# Installation

## IQ_SIM
Simply clone our fork:
```
git clone https://github.com/davidtr99/iq_sim
```


## IQ_GNC
Simply clone our fork:
```
git clone https://github.com/davidtr99/iq_gnc
```
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