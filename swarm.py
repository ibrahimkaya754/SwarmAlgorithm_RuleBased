# Import Modules
import numpy as np
import pygame
import random
import math
from matplotlib import pyplot as plt

class swarm():
    def __init__(self, screensize, target_location, number_of_particles=20, display=False, dim=2, 
                       CommRng= 200, delta_t= 0.1, TimeConstant=0.01, summary=True, import_coefficients=False,
                       params=np.zeros(14)):
        self.screensize   = screensize
        self.nop          = number_of_particles
        self.display      = display
        self.summary      = summary
        self._vel_max     = 20.0
        self._vflock      = 20.0
        self.member       = {}
        self.iteration_no = 0
        self.dim          = 2        # 2 dimensional motion
        self.CommRng      = CommRng  # Communication Range
        self.delta_t      = delta_t
        self.TimeConstant = TimeConstant
        self.trgt_loc     = {str(ii): target_location[ii] for ii in range(self.dim)}
        self.random       = {str(jj): np.random.randint(0,2) for jj in range(self.dim)} 
        self.params       = params
        
        self.import_coefficients = import_coefficients
        self.__coefficients_()
        
        self.wght         = {'follower': [self.params[ii] for ii in range(8,14)],
                             'leader'  : np.array((0.0,0.0,0.0,0.0,10.0,0.0)),
                             'rlagent' : [self.params[ii] for ii in range(8,14)]}
        if self.display:
            self.screen       = pygame.display.set_mode((self.screensize[0],self.screensize[1]))
            pygame.display.set_caption("Swarm System")
            self.screen.fill(self.WHITE)
                  
        for ii in range(self.nop):
            self.member[str(ii)] = {'center':(0,0)}            
            self.member[str(ii)]['position']    = {str(jj): np.random.random() * self.screensize[jj] * 0.2 \
                                                            + self.random[str(jj)] * self.screensize[jj] * 0.8 \
                                                            for jj in range(self.dim)}
            self.member[str(ii)]['velocity']    = {str(jj): (1-self.random[str(jj)])*np.random.random()*20 - self.random[str(jj)]*20 for jj in range(self.dim)}
            self.member[str(ii)]['deltavel']    = {str(jj): 0 for jj in range(self.dim)}
            self.member[str(ii)]['deltapos']    = {str(jj): 0 for jj in range(self.dim)}
            self.member[str(ii)]['center']      = (int(np.round(self.member[str(ii)]['position']['0'])),
                                                   int(np.round(self.member[str(ii)]['position']['1'])))
            self.member[str(ii)]['algo']        = {'rulebased': True,
                                                   'rl'       : False}                
            self.member[str(ii)]['role']        = 'follower'
            self.member[str(ii)]['target']      = 'leader'
            self.member[str(ii)]['PrtclsInRng'] = 0
        self.leader, self.rlagent               = [str(ii) for ii in range(2)]
        self.member[self.leader]['role']        = 'leader'
        self.member[self.leader]['target']      = 'target'
        self.member[self.rlagent]['role']       = 'follower'
        self.targetposition                     = {'leader': self.member[self.leader]['position'],
                                                   'target': self.trgt_loc}
        self.member[self.rlagent]['algo']       = {'rulebased': True,
                                                   'rl'       : False}
        self.color                              = {'leader'   : self.RED,
                                                   'follower' : self.BLUE,
                                                   'rlagent'  : self.BLUE}
        self.__distance()
        if self.summary:
            print('\n-----------------------')
            print('SUMMARY FOR PARTICLES')
            print('-----------------------\n')
            for key in self.member.keys():
                print('particle id : ', key)
                print('role        : ',self.member[key]['role'])
                print('target      : ',self.member[key]['target'])
                print('wghts       : ',self.wght[self.member[key]['role']])
                print('dist2wp     : ',self.member[key]['distance2target'])
                print('-----------------------')
                
    def __coefficients_(self):
        if not self.import_coefficients: 
            self.params = [19.087,77.570,74.741,49.385,50.461,31.121,715.096,
                           87.658,-8.527,9.441,-1.126,7.908,5.326,-3.060]
        self.BLUE   = (0,0,255)
        self.WHITE  = (255,255,255)
        self.BLACK  = (0,0,0)
        self.RED    = (255,0,0)
            
    def __distance(self):
        '''calculates the distances between the members of the swarm'''
        for ii in range(self.nop):
            self.member[str(ii)]['distance']            = {str(jj): {str(kk): self.member[str(ii)]['position'][str(kk)]-\
                                                                              self.member[str(jj)]['position'][str(kk)] \
                                                                              for kk in range(self.dim)} \
                                                                    for jj in range(self.nop)}
            
            self.member[str(ii)]['distance_sorted']     = {str(jj): sorted(np.abs(self.member[str(ii)]['distance'][str(kk)][str(jj)])\
                                                                    for kk in range(self.nop)) for jj in range(self.dim)}
            
            self.member[str(ii)]['abs_distance']        = {str(jj): np.sqrt(math.fsum(self.member[str(ii)]['distance'][str(jj)][str(kk)]**2\
                                                                    for kk in range(self.dim))) for jj in range(self.nop)}
            
            self.member[str(ii)]['abs_distance_sorted'] = [sorted(self.member[str(ii)]['abs_distance'].items(), \
                                                           key = lambda kv:(kv[1], kv[0]))[jj][1] for jj in range(self.nop)]
            
            self.member[str(ii)]['closest_neighbours']  = [sorted(self.member[str(ii)]['abs_distance'].items(), \
                                                           key = lambda kv:(kv[1], kv[0]))[jj][0] for jj in range(self.nop)]  

            self.member[str(ii)]['distance2target']     = (lambda x: np.sqrt(x[0]**2+x[1]**2))\
                                                          ([self.member[str(ii)]['position'][str(jj)] -\
                                                          self.targetposition[self.member[str(ii)]['target']][str(jj)] \
                                                          for jj in range(self.dim)])
            ### Relative Velocity and Position are found according to the closest neighbours and the leader                
            self.member[str(ii)]['relative_velocity']   = {neighbor: {str(kk): self.member[str(ii)]['velocity'][str(kk)]-\
                                                                               self.member[neighbor]['velocity'][str(kk)] \
                                                                               for kk in range(self.dim)} \
                                                                     for neighbor in self.member[str(ii)]['closest_neighbours'][1:]}

            self.member[str(ii)]['relative_position']   = {neighbor: {str(kk): self.member[str(ii)]['position'][str(kk)]-\
                                                                               self.member[neighbor]['position'][str(kk)] \
                                                                               for kk in range(self.dim)} \
                                                                     for neighbor in self.member[str(ii)]['closest_neighbours'][1:]}
                                    
            self.member[str(ii)]['distance2leader']     = {str(kk): self.member[str(ii)]['position'][str(kk)]-\
                                                                    self.member[self.leader]['position'][str(kk)] \
                                                                    for kk in range(self.dim)}

            self.member[str(ii)]['velocity2leader']     = {str(kk): self.member[str(ii)]['velocity'][str(kk)]-\
                                                                    self.member[self.leader]['velocity'][str(kk)] \
                                                                    for kk in range(self.dim)}
    
    ### Rule Based Algo based on Potential Field 170820 ###
    def rulebasedalgo(self):       
        for axis in range(self.dim):
            for particle in self.member.keys():
                if self.member[particle]['role'] != 'rlagent':
                    r0,r1,r2,D,Cfrict,Cshill,R,d         = [self.params[ii] for ii in range(0,8)]
                    apot, aslp, v1, a1, a6               = [0 for ii in range(5)]
                    self.member[particle]['PrtclsInRng'] = 3 if len([ii for ii in self.member[particle]['abs_distance_sorted'] if ii<r0]) < 3 else \
                                                                len([ii for ii in self.member[particle]['abs_distance_sorted'] if ii<r0])# the number of neigbours closer than r0
                    for neigbour in self.member[particle]['closest_neighbours'][1:self.member[particle]['PrtclsInRng']]:
                        if np.abs(self.member[neigbour]['position'][str(axis)] - self.member[particle]['position'][str(axis)]) < r0:
                            apot = apot + D * np.minimum(r1,r0 - np.abs(self.member[neigbour]['position'][str(axis)] - \
                            self.member[particle]['position'][str(axis)])) * (self.member[neigbour]['position'][str(axis)]  - \
                            self.member[particle]['position'][str(axis)]) / np.abs(self.member[neigbour]['position'][str(axis)] - \
                            self.member[particle]['position'][str(axis)])
                        else:
                            apot = 0

                        aslp = aslp + Cfrict*(self.member[neigbour]['velocity'][str(axis)] - \
                            self.member[particle]['velocity'][str(axis)]) / \
                            (np.maximum(np.abs(self.member[neigbour]['position'][str(axis)] - \
                            self.member[particle]['position'][str(axis)]) - (r0-r2),r1))**2

                        a1   = a1 + (self.member[neigbour]['position'][str(axis)] - \
                            self.member[particle]['position'][str(axis)]) / self.member[particle]['PrtclsInRng']

                        v1   = v1 + (self.member[neigbour]['velocity'][str(axis)] - \
                            self.member[particle]['velocity'][str(axis)]) / self.member[particle]['PrtclsInRng']

                    vtrack = 0
                    a6     = self.targetposition[self.member[particle]['target']][str(axis)] - self.member[particle]['position'][str(axis)]
                    vspp   = self._vflock * self.member[particle]['velocity'][str(axis)] / np.abs(self.member[particle]['velocity'][str(axis)])
                    awall  = Cshill * self.smooth_transfer_function(np.abs(self.targetposition[self.member[particle]['target']][str(axis)] - \
                            self.member[particle]['position'][str(axis)]),R,d) * (self._vflock * ((self.targetposition[self.member[particle]['target']][str(axis)] - \
                            self.member[particle]['position'][str(axis)]) / np.abs(self.targetposition[self.member[particle]['target']][str(axis)] - \
                            self.member[particle]['position'][str(axis)])) - self.member[particle]['velocity'][str(axis)])
                    
                    self.member[particle]['deltavel'][str(axis)] = self.wght[self.member[particle]['role']][5] * (v1+vspp+vtrack-self.member[particle]['velocity'][str(axis)]) + \
                                                                   self.wght[self.member[particle]['role']][0] * apot + self.wght[self.member[particle]['role']][1] * aslp + \
                                                                   self.wght[self.member[particle]['role']][2] * awall + \
                                                                   self.wght[self.member[particle]['role']][3] * a1 + self.wght[self.member[particle]['role']][4] * a6
 
    ### Velocity Update ###
    def update(self,keepGoing):
        self.keepGoing = keepGoing
        for axis in range(self.dim):
            for particle in self.member.keys():
                
                try:
                    self.member[particle]['velocity'][str(axis)] = self.member[particle]['velocity'][str(axis)] + \
                                                                   self.member[particle]['deltavel'][str(axis)] * self.delta_t
                                  
                    if self.member[particle]['velocity'][str(axis)] > self._vel_max:
                        self.member[particle]['velocity'][str(axis)] = self._vel_max
                    elif self.member[particle]['velocity'][str(axis)] < -self._vel_max:
                        self.member[particle]['velocity'][str(axis)] = -self._vel_max
    
                    if self.member[particle]['distance2target'] <= 100:
                        self.member[particle]['velocity'][str(axis)] = self.member[particle]['distance2target']/200 *self.member[particle]['velocity'][str(axis)]
                    
                    self.member[particle]['deltapos'][str(axis)] = self.member[particle]['velocity'][str(axis)] * self.delta_t
                    self.member[particle]['position'][str(axis)] = self.member[particle]['position'][str(axis)] + self.member[particle]['deltapos'][str(axis)]
                    self.member[particle]['center']              = (int(np.round(self.member[particle]['position']['0'])),
                                                                    int(np.round(self.member[particle]['position']['1'])))
                    
                except:
                    self.member[particle]['position']    = {str(jj): np.random.random() * self.screensize[jj] * 0.2 \
                                                                    + self.random[str(jj)] * self.screensize[jj] * 0.8 \
                                                                    for jj in range(self.dim)}
                    self.member[particle]['velocity']    = {str(jj): (1-self.random[str(jj)])*np.random.random()*20 - self.random[str(jj)]*20 for jj in range(self.dim)}
                    self.member[particle]['deltavel']    = {str(jj): 0 for jj in range(self.dim)}
                    self.member[particle]['deltapos']    = {str(jj): 0 for jj in range(self.dim)}
                    self.member[particle]['center']      = (int(np.round(self.member[particle]['position']['0'])),
                                                            int(np.round(self.member[particle]['position']['1'])))
                
                pygame.draw.circle(self.screen,self.color[self.member[particle]['role']],self.member[particle]['center'],2)
                
        if self.keepGoing:
            self.__display()
            self.__distance()

    def __display(self):
        if self.display:
            pygame.display.flip()     
            self.screen.fill(self.WHITE)
    


    ######################### SMOOTH TRANSFER FUNCTION ########################################################################################
    def smooth_transfer_function(self,x,R,d):
        if x<=R:
            res = 0
        elif x>R and x<=R+d:
            res = (np.sin(np.pi/d*(x-R) - np.pi/2) + 1) / 2
        else:
            res = 1
        return res