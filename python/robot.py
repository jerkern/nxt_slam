""" Module implementing robot motion models """

from __future__ import division
import math
import numpy

class DifferentialRobot(object):
    """ Differential drive robot """

    def __init__(self, l, d, ticks=1000, state=(0.0, 0.0, math.pi/2),
                 wp=(0.0, 0.0)):
        """ Create robot object which can be used to simulate movement 
        
        l - wheel base distance
        d - wheel diameter
        ticks - ticks per revolution for the wheel encoders
        state - initial state (x,y,theta)
        wp - initial positions for the wheel positions (l,r)
        
        """
            
        assert(l > 0)
        assert(d > 0)
        
        self.l = l
        self.d = d
        self.state = numpy.asarray(state)
        self.old_S = sum(wp)
        self.old_D = numpy.diff(wp)[0]
        self.old2_S = self.old_S
        self.old2_D = self.old_D
        self.ticks = ticks

        # This contant is needed in the update phase        
        self.C = math.pi * self.d / self.ticks

    def kinematic(self, wp):
        """ Update state using the new wheel positions

            wp - wheel positions """

     
        #S = numpy.sum(wp)
        S = wp[0]+wp[1]
        #D = numpy.diff(wp)[0]
        D = wp[1]-wp[0]
        
        theta_new = self.calc_next_theta(wp)
        
        tmp = self.calc_next_xy(wp, theta_new)
        self.state[0] = tmp[0]
        self.state[1] = tmp[1]
        self.state[2] = theta_new
        
        self.old2_S = self.old_S
        self.old_S = S
        
        self.old2_D = self.old_D
        self.old_D = D
        
        # Return estimate of how robot has moved (forward, turn)
        return (self.C/2.0*(self.old_S - self.old2_S), 
                self.C/(2.0*self.l)*(self.old_D - self.old2_D))
        
    def calc_next_theta(self, wp):
        # Tustion approximation for the state
        # Velocities estimated using backward difference           
        #D = numpy.diff(wp)[0]
        D = wp[1] - wp[0]
        
        # First estimate new angle, since it doesn't depend on other states
        # and is needed for the other calculations        
        theta_new = self.state[2] + self.C/(2.0*self.l)*(D-self.old2_D)
        
        return theta_new
        
    def calc_next_xy(self, wp, theta_new):
        
        #S = numpy.sum(wp)
        S = wp[0] + wp[1]
        #D = numpy.diff(wp)[0]
        D = wp[1] - wp[0]
        
        cn = math.cos(theta_new)
        sn = math.sin(theta_new)
        
        co = math.cos(self.state[2])
        so = math.sin(self.state[2])
        
#        next_xy = numpy.copy(self.state[:2]) 
#        
#        next_xy[0] += self.C/4*((S-self.old_S)*cn+
#                                (self.old_S-self.old2_S)*co)
#        next_xy[1] += self.C/4*((S-self.old_S)*sn+
#                                (self.old_S-self.old2_S)*so)
#        return next_xy
        
        return numpy.array((self.state[0]+self.C/4*((S-self.old_S)*cn+(self.old_S-self.old2_S)*co),
                            self.state[1]+self.C/4*((S-self.old_S)*sn+(self.old_S-self.old2_S)*so)))
    
    def set_state(self, x, y, theta):
        self.state[0] = x
        self.state[1] = y
        self.state[2] = theta
        
        
    def add_state_noise(self, os):
        # Noise specified in local coordinate system, transform to global first
                # Add offset of sensor, offset is specified in local coordinate system,
        # rotate to match global coordinates
        theta = self.state[2]
        rot_mat = numpy.array(((math.cos(theta), -math.sin(theta)),
                    (math.sin(theta), math.cos(theta))))
        
        self.state[:2] = self.state[:2] + rot_mat.dot(os[:2])
        
        self.state[2] = self.state[2] + os[2]

