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

## Installation
\todo
