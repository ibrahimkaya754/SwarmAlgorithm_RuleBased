"""
Created on Tue Feb 18 09:50:14 2020

@author: ibrahimkaya

The Rulebased Algorithm has been taken from the great paper 
"Outdoor flocking and formation flight with autonomous aerial robots" by Vicsek et.al.
"""
import numpy as np
import math

class swarm_algorithm():
    def __init__(self,params):
        self.params                     = params
        
    def algo(self,particle,axes,position,velocity,closest_particles_abs,
             xtrg,wght,distance_to_target,vflock= 20,delta_t= 0.01,
             vel_min= -20.0,vel_max= 20.0,TimeConstant=0.01):
        
        r0,r1,r2,D,Cfrict,Cshill,R,d    = [self.params[ii] for ii in range(8)]
        apot, aslp, v1, a1, a6          = [0 for ii in range(5)]
    
        self.particle                   = particle
        self.axes                       = axes
        self.position                   = position
        self.velocity                   = velocity
        self.closest_particles_abs      = closest_particles_abs
        self.xtrg                       = xtrg
        self.wght                       = wght
        self.distance_to_target         = distance_to_target
        self.closestneighbour_number    = 3

        for nb in range(1,self.closestneighbour_number):
            if np.abs(self.position[self.closest_particles_abs[self.particle,nb],self.axes] - self.position[self.particle,self.axes]) < r0:
                apot = apot + D * np.minimum(r1,r0 - np.abs(self.position[self.closest_particles_abs[self.particle,nb],self.axes] - \
                       self.position[self.particle,self.axes])) * (self.position[self.closest_particles_abs[self.particle,nb],self.axes]  - \
                       self.position[self.particle,self.axes]) / np.abs(self.position[self.closest_particles_abs[self.particle,nb],self.axes] - \
                       self.position[self.particle,self.axes])
            else:
                apot = apot + 0

            aslp = aslp + Cfrict * (self.velocity[self.closest_particles_abs[self.particle,nb],self.axes] - \
                   self.velocity[self.particle,self.axes]) / (np.maximum(np.abs(self.position[self.closest_particles_abs[self.particle,nb],self.axes] - \
                   self.position[self.particle,self.axes])-(r0-r2),1))**2

            a1   = a1 + (self.position[self.closest_particles_abs[self.particle,nb],self.axes] - \
                   self.position[self.particle,self.axes]) / self.closestneighbour_number

            a6   = a6 + self.xtrg[self.axes] - self.position[self.particle,self.axes]

            v1   = v1 + (self.velocity[self.closest_particles_abs[self.particle,nb],self.axes] - \
                   self.velocity[self.particle,self.axes]) / self.closestneighbour_number

        vspp   = vflock * self.velocity[self.particle,self.axes] / np.abs(self.velocity[self.particle,self.axes])
        awall  = Cshill * self.smooth_transfer_function(np.abs(self.xtrg[self.axes] - \
                self.position[self.particle,self.axes]),R,d) * (vflock * ((self.xtrg[self.axes] - \
                self.position[self.particle,self.axes]) / np.abs(self.xtrg[self.axes] - \
                self.position[self.particle,self.axes])) - self.velocity[self.particle,self.axes])
        vtrack = 0
        
        self.delta_vel = ((self.wght[5]) * (v1+vspp+vtrack-self.velocity[self.particle,self.axes]) + \
                    (self.wght[0] * apot + self.wght[1] * aslp + self.wght[2] * awall + self.wght[3] * a1 + self.wght[4] * a6))
        
        self.velocity[self.particle,self.axes] = self.velocity[self.particle,self.axes] + self.delta_vel * delta_t
        self.velocity[self.particle,self.axes] = self.distance_to_target/100 * self.velocity[self.particle,self.axes]
        
        if self.velocity[self.particle,self.axes]>vel_max:
            self.velocity[self.particle,self.axes] = vel_max
        elif self.velocity[self.particle,self.axes]<vel_min:
            self.velocity[self.particle,self.axes] = vel_min
       
        self.position_delta = self.velocity[self.particle,self.axes] * delta_t

  

        # if self.axes == 0:
        #     if self.delta_vel > 10:
        #         self.delta_vel = 10
        #     elif self.delta_vel < -10:
        #         self.delta_vel = -10

        #     #We can omit the time delays in acceleration in x-direction (where thrust is applied)
        #     #self.delta_vel = self.delta_vel * (1-math.exp(-delta_t/TimeConstant)) #dynamic with 1st order system response
        #     self.velocity[self.particle,self.axes] = self.velocity[self.particle,self.axes] + self.delta_vel * delta_t
            
        #     if self.velocity[self.particle,self.axes]>vel_max:
        #         self.velocity[self.particle,self.axes] = vel_max
        #     elif self.velocity[self.particle,self.axes]<vel_min:
        #         self.velocity[self.particle,self.axes] = vel_min
        # else:
        #     if self.delta_vel > 10:
        #         self.delta_vel = 10
        #     elif self.delta_vel < -10:
        #         self.delta_vel = -10 

        #     self.delta_vel = self.delta_vel * (1-math.exp(-delta_t/TimeConstant)) #dynamic with 1st order system response
        #     self.velocity[self.particle,self.axes] = self.velocity[self.particle,self.axes] + self.delta_vel * delta_t

        #     if self.velocity[self.particle,self.axes]>vel_max:
        #         self.velocity[self.particle,self.axes] = vel_max
        #     elif self.velocity[self.particle,self.axes]<-vel_max:
        #         self.velocity[self.particle,self.axes] = -vel_max
       
        # if self.distance_to_target <= 100:
        #     self.velocity[self.particle,self.axes] = self.distance_to_target/200 * self.velocity[self.particle,self.axes]
        #self.position_delta = self.velocity[self.particle,self.axes] * delta_t
    
    ######################### SMOOTH TRANSFER FUNCTION ########################################################################################
    def smooth_transfer_function(self,x,R,d):
        if x<=R:
            res = 0
        elif x>R and x<=R+d:
            res = (np.sin(np.pi/d*(x-R) - np.pi/2) + 1) / 2
        else:
            res = 1
        return res
