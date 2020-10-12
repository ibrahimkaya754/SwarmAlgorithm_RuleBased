'''
Author: ikaya
'''
#%%
# Import neccessary modules
import time
import math
import matplotlib.pyplot as plt
import pygame
import random
import numpy as np
from pygame.locals import *
from swarm import *
from pso.PSO import swarm as opt
################################################################################################
#%%
# Simulation Parameters
number_of_particles = 30
number_of_axes      = 2
delta_t             = 0.2
t_final             = 100
screen_size         = [3000,1800]
initial_location    = [screen_size[0]/2,screen_size[1]/2]
list_min_distance   = []
list_ave_distance   = []
xtrg                = [initial_location[ii] + np.random.randint([900,1400])[ii] for ii in range(number_of_axes)]
particles           = swarm(number_of_particles=number_of_particles, screensize=screen_size, target_location=xtrg,
                          display=True, CommRng=100, dim=number_of_axes, delta_t= delta_t)
leader              = particles.leader
numberofneighbour   = 2
numberofleader      = 1
clock               = pygame.time.Clock()
numberofepochs      = 1

### The multiplayer 2 below is for 'position' and 'velocity' ###
print('----------------------------------------------------------------------------')
print('There will be %s states, %s for relative velocity, %s for relative position' % \
      (particles.dim*(numberofneighbour+numberofleader)*2,\
       particles.dim*(numberofneighbour+numberofleader),\
       particles.dim*(numberofneighbour+numberofleader)))
print('----------------------------------------------------------------------------')
################################################################################################
#%%
def function2opt(params):
    particles.import_coefficients = True
    particles.params = params
    t                = 0 
    mean_closest     = np.zeros(number_of_particles)
    mean_furthest    = np.zeros(number_of_particles)
    mean2target      = np.zeros(number_of_particles)
    val2min          = []
    
    while t<=t_final:
        particles.rulebasedalgo()
        particles.update(keepGoing=True)
        print('time: %.2f' % (t))
        t = t + delta_t
    
        if t%t_final >= 0.0 and t%t_final < delta_t:
            print('\ntarget location changes\n')
            particles.trgt_loc                 = {str(ii) : np.random.randint(screen_size)[ii] for ii in range(particles.dim)}
            particles.targetposition['target'] = particles.trgt_loc
    
        for ii in range(particles.nop):
            mean_closest[ii]  = np.abs(150.0-particles.member[str(ii)]['abs_distance_sorted'][1])
            mean_furthest[ii] = np.abs(1000.0-particles.member[str(ii)]['abs_distance_sorted'][-1])
            mean2target[ii]   = particles.member[str(ii)]['distance2target']
            
        val2min.append(np.mean(mean_closest) + np.mean(mean_furthest)+ np.mean(mean2target))
        
    return np.mean(val2min)
################################################################################################
#%%
xtrg         = [np.random.randint(screen_size)[ii] for ii in range(number_of_axes)]
optimization = opt(func=function2opt,lowerbounds=-100*np.ones(14),upperbounds=100*np.ones(14),number_of_particles=60,readsavedfile=[True,'best_vals_ever.txt'])
particles.__init__(number_of_particles=number_of_particles, screensize=screen_size, target_location=xtrg, display=True, CommRng=100, summary=False)
################################################################################################
#%%
optimization.update(iteration=50)
#%%  Save the best values
best_vals_list = []
for elem in list(optimization.best_position.keys()): 
    best_vals_list.append(list(optimization.member[elem]['best_position'].values()))

best_vals_ever_list = []
for elem in list(optimization.best_position_ever.keys()): 
    best_vals_ever_list.append(list(optimization.member[elem]['best_position'].values()))
    
np.savetxt('best_vals.txt',best_vals_list)
np.savetxt('best_vals_ever.txt',best_vals_ever_list)
################################################################################################
#%%                
for key in particles.member.keys():
    print('Particle id       : %s' % (key))
    print('Particle role     : %s' % (particles.member[key]['role']))
    print('Particle color    : ', particles.color[particles.member[key]['role']])
    print('Particle target   : %s' % (particles.member[key]['target']))
    print('particle velocity : %s' % (particles.member[key]['velocity']))
    print('particle position : %s' % (particles.member[key]['position']))
    print('target position   : %s' % (particles.targetposition[particles.member[key]['target']]))
    print('weigths           : %s' % (particles.wght[particles.member[key]['role']]))
    print('particles in rng  : %s' % (particles.member[key]['PrtclsInRng']))
    print('------------------------------------')
################################################################################################
# %%
print('best_particle      : ',list(optimization.best_position.keys())[0])
print('best_position      : ',optimization.member[list(optimization.best_position.keys())[0]]['best_position'])
print('best_value         : ',optimization.member[list(optimization.best_position.keys())[0]]['fitness_value'])
print('best_particle_ever : ',list(optimization.best_position_ever.keys())[0])
print('best_position_ever : ',optimization.member[list(optimization.best_position_ever.keys())[0]]['best_position'])
print('best_value_ever    : ',optimization.member[list(optimization.best_position_ever.keys())[0]]['best_value'])
print('best_value_obtained at iteration no: %s' % (optimization.best_value_iteration))
################################################################################################
# %%
best_vals_ever = list(optimization.member[list(optimization.best_position.keys())[0]]['best_position'].values())
function2opt(params=best_vals_ever)
################################################################################################
# %%
mean_closest     = np.zeros(number_of_particles)
mean_furthest    = np.zeros(number_of_particles)
for ii in range(particles.nop):
    mean_closest[ii]  = particles.member[str(ii)]['abs_distance_sorted'][1]
    mean_furthest[ii] = particles.member[str(ii)]['abs_distance_sorted'][-1]
    
################################################################################################
