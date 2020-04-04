#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 13 13:25:16 2017

@author: roketsan
"""
import numpy as np
import pygame
import random

class particle(pygame.sprite.Sprite): # CUSTOMIZABLE OBJECT
    def __init__(self, screen, background, color):
        pygame.sprite.Sprite.__init__(self)
        self.screen = screen
        self.background = background
        if color == 0:
            color1 = 200
            color2 = 100
            color3 = 50 
        elif color == 1:
            color1 = 200
            color2 = 100
            color3 = 50 
        elif color == 2:
            color1 = 10
            color2 = 10
            color3 = 10            
        self.image = pygame.Surface((4, 4))
        self.image.fill((255, 255, 255))
        pygame.draw.circle(self.image, (color1, color2, color3), (2, 2), 2) 
        self.rect = self.image.get_rect()       
        
        self.positionx      = 500+np.random.random()*10
        self.positiony      = 5+np.random.random()*10
        self.rect.center    = (self.positionx, self.positiony)
        self.vel_min        = -20
        self.vel_max        = 20
        self.lower_boundary = np.array([-1500,-1500])
        self.upper_boundary = np.array([1500,1500])
        self.velx           = np.random.rand()
        self.vely           = np.random.rand()
        self.dx             = random.random() * 20
        self.dy             = random.random() * 20
     
    def update(self): 
#        oldCenter = self.rect.center
#        self.rect.centerx += self.dx #x_hareketi
#        self.rect.centery += self.dy #y_hareketi
        self.positionx      = self.positionx + self.dx
        self.positiony      = self.positiony + self.dy
        self.rect.center    = (self.positionx, self.positiony)
#        pygame.draw.line(self.background, (0, 0, 0), oldCenter, self.rect.center) # iz        
        self.checkBounds()        
    
    def checkBounds(self):
        """ bounce on encountering any screen boundary """
        
        if self.rect.right >= self.screen.get_width():
            self.dx *= -1
        if self.rect.left <= 0:
            self.dx *= -1
        if self.rect.bottom >= self.screen.get_height():
            self.dy *= -1
        if self.rect.top  <= 0:
            self.dy *= -1
            
###################### FIND DISTANCES ###################################################################################################
class distance():
    def __init__(self,population_number,dimension,distances,
                 distances_abs,position,closest_distances_abs,
                 closest_particles_abs,closestneighbours):
        self.population_number      = population_number
        self.dimension              = dimension
        self.distances              = distances
        self.distances_abs          = distances_abs
        self.position               = position
        self.closest_distances_abs  = closest_distances_abs
        self.closest_particles_abs  = closest_particles_abs
        self.closestneighbours      = closestneighbours

    def find_distances(self):
        self.closest_neighbours  = np.zeros((self.population_number,self.population_number))
        for jj in range(self.dimension):
            for ii in range(self.population_number):
                for kk in range(self.population_number):
                    self.dim               = jj * self.population_number + kk
                    self.distances[ii,self.dim] = self.position[ii,jj] - self.position[kk,jj]

        for ii in range(self.population_number):
            for kk in range(self.population_number):
                column1                   = kk
                column2                   = self.population_number + kk
                self.distances_abs[ii,kk] = np.sqrt(self.distances[ii,column1]**2 + self.distances[ii,column2]**2)

        for ii in range(self.population_number):
            self.closest_distances_abs[ii,:]       = np.sort(self.distances_abs[ii,:])
            self.closest_particles_abs[ii,:]       = np.argsort(self.distances_abs[ii,:])

        for ii in range(self.population_number):
            for kk in range(self.population_number):
                self.closest_neighbours[ii,kk] = self.closest_distances_abs[ii,kk] < 30     # 30 m has been chosen randomly, must be optimized later.

        for ii in range(self.population_number):
            self.closestneighbours[ii]   = 0
            for kk in range(self.population_number):
                if self.closest_neighbours[ii,kk] == True:
                    self.closestneighbours[ii] = self.closestneighbours[ii] + 1
                else:
                    break
        
        return self.distances,self.distances_abs,self.closest_distances_abs,self.closest_particles_abs,self.closestneighbours

    def distance_to_waypoint(self,waypoint):
        number_of_particles = self.population_number - 1
        dist_to_wp = np.zeros((number_of_particles,1))
        for ii in range(number_of_particles):
            dist             = self.position[ii,:] - waypoint
            dist_to_wp[ii,0] = np.sqrt(dist[0]**2 + dist[1]**2)
        return dist_to_wp      

    def dist_to_wypnt(self,pos,waypoint):
        dist       = pos - waypoint
        dist_to_wp = np.sqrt(dist[0]**2 + dist[1]**2)
        return dist_to_wp  
        
###########################################################################################################################################