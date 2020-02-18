# SwarmAlgorithm_RuleBased

Swarm algorithm by using the great paper "Outdoor flocking and formation flight with autonomous aerial robots" by Vicsek et.al.

To run the code, pygame should be installed to the environment simply by using pip (pip install pygame).


Swarm Algorithm for Objects Moving in 2D Domain:
The swarm concept in recents years has gain quite popularity in many applications, especially in defence industry. The most attracting side of swarm concept is to be able to fulfill complex duties with relatively cheap ammo.

The code below shows swarming particles (like drones) in 2 Dimensional domain and has been generated with rule based algorithm given in the great paper "Outdoor flocking and formation flight with autonomous aerial robots" by Vicsek et. al. [1] The document referenced in [2] is also a very explanatory and beneficial to understand the basics of swarm concept.

The first thing to do is to import necessary packages to our environment.

Main Parameters:
The parameters given below inside the main function has been obtained by Genetic Algorithm (GA). The writers of the paper did not give the values of the parameters, which is quite reasonable to some point, since every application has its own dynamics, i.e., the parameters from one applicastion to the other will probably be very different. For my application, I used GA code I have shared in my github repo [3] and the algorithm has given the parameters seen inside the main function.

There are 2 important modules imported to our environment above. One of them is PygameModule which generates the neccessary things for visualization of the particles moving with swarm condition. The other one is the Swarm_Algoritm_RuleBased, where the rulebased algorithm given in the paper is coded.

What is happenning in the simulation can be summarized as the following:

There are 51 particles swarming in the domain

One of them is the leader that every information about the mission is known by

The other 50 particles does not know anything about the mission but the only thing they know is they should follow the leader without creating any crash inside the population

The points of the target that is to be followed were given in the list of waypoints, where it can be changed inside the domain randomly.

The parameters obtained by GA has been generated for a fitness function minimizing the sum of the distances between the group members (minimization of the dispersion), while preventing the individual particles from approaching to eachother greater than a value defined (8 meters given in our case).

Although the number of total particles (population) has been given as 51, it can be increased (to 100s) or decreased (to around 10) since the algorithm coded can be applied for any number of particles, i.e., the algorithm given here is scalable.

# Import neccessary modules
import time
import math
import matplotlib.pyplot as plt
import pygame
import random
import numpy as np
from pygame.locals import *
from PygameModule import *
from Swarm_Algorithm_RuleBased import *

Refences¶
[1] "Outdoor flocking and formation flight with autonomous aerial robots", G. Vásárhelyi, Cs. Virágh, G. Somorjai, N. Tarcai, T. Szörényi, T. Nepusz, T. Vicsek

[2] "Dynamic Mission Control for UAV Swarm via Task Stimulus Approach", Haoyang Cheng , John Page, John Olsen

[3] https://github.com/ibrahimkaya754/GeneticAlgorithm
