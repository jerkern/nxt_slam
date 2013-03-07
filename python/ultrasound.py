""" Module for working with ultrasonic distance sensors and occupancy grids """
import numpy as np
import math
import copy
import scipy.stats
from occupancy_grid import OccupancyGrid

class UltrasoundRobot(object):
    """ Robot using ultrasonic distance measurement for navigation """
    def __init__(self, offset, prec, rob, enc_noise, unknown, decay, bounds=None,
                 resolution=None, cov_offset = 1.0, a_coeff=1.0, r_coeff=1.0,
                 prior=0.5, num_angles=1, detect_range=np.Inf):
        self.offset = np.asarray(offset)
        if (bounds == None):
            bounds = (-5.0, 5.0, -5.0, 5.0)
        if (resolution == None):
            resolution = 0.4
        self.map = OccupancyGrid(bounds=bounds, resolution=resolution,
                                 unknown=unknown, decay=decay, prior=prior,
                                 num_angles=num_angles)
        self.robot = copy.deepcopy(rob)
        self.prec = prec
        self.cov_offset = cov_offset
        self.a_coeff = a_coeff
        self.r_coeff = r_coeff
        self.num_angles = num_angles
        self.detect_range = detect_range
        self.enc_noise = enc_noise

    def update(self, data):
        """ Move robot according to kinematics """
        self.motion = self.robot.kinematic(data[:2])
        self.set_sensor_angle(data[2])
        #self.add_state_noise(data[3:])
        self.map.time_update()

    def sample_input_noise(self, w):
        
        w = np.asarray(w)
        
        wn = np.random.uniform(w[:2,:]-self.enc_noise/2, w[:2,:]+self.enc_noise/2, (2,1))
        
        
        return np.hstack((wn.ravel(), w[2,:]))
            
    def add_state_noise(self, os):
        self.robot.add_state_noise(os)
    
    def sample_input_noise(self, w):
        
        w = np.asarray(w)
        
        wn = np.random.uniform(w[:2,:]-self.enc_noise/2, w[:2,:]+self.enc_noise/2, (2,1))
        
        return np.hstack((wn.ravel(), w[2,:]))
            
    def measure(self, dist):
        """ Process new distance measurement """

        # Add offset of sensor, offset is specified in local coordinate system,
        # rotate to match global coordinates
        state = np.copy(self.robot.state)
        theta = state[2]
        rot_mat = np.array(((math.cos(theta), -math.sin(theta)),
                    (math.sin(theta), math.cos(theta))))
        
        state[:2] = state[:2] + rot_mat.dot(self.offset[:2])
        
        # We can just add the angle, since unwrapping of
        # angles is handled inside FOVCone
        state[2] = state[2] + self.offset[2]
        
        cell_diag = 1.44*self.map.resolution
        # If outside "detect_range" don't detect anything, used update
        # the empty area in front
        norm = True
        if (dist > self.detect_range):
            cell_diag = -cell_diag
            norm = False
            
        # Limit max distance of updates
        cone = FOVCone(state[:2], state[2], self.prec, min(dist+cell_diag, 15.0))
        
        # To improve performance we just extract which cells are within
        # the FOV once, and reuse this both for measure and update.
        cells = self.map.find_visible_cells(cone)
        
        if (cells != None):
            # Create probability field for measurement
            sonar_measure = SonarMeasurement(cells, dist, self.map.resolution/2,
                                             r_coeff=self.r_coeff,
                                             a_coeff=self.a_coeff,
                                             cov_offset=self.cov_offset,
                                             num_angles=self.num_angles,
                                             norm=norm)
            
            # Eval measurement against map and update
            return self.map(sonar_measure)
        
        else:
            return 0


    def set_sensor_angle(self, theta):
        """ Move sensor to specified angle """
        self.offset[2] = theta


    
class SmoothUltrasoundRobot(UltrasoundRobot):
    """ Robot using ultrasonic distance measurement for navigation """
    def __init__(self, offset, prec, rob, unknown, decay, state_noise_off=None,
                 state_noise_1st=None, bounds=None, resolution=None,
                 cov_offset = 1.0, a_coeff=1.0, r_coeff=1.0, prior=0.5,
                 num_angles=1, detect_range=np.Inf):
        super(SmoothUltrasoundRobot, self).__init__(offset, prec, rob, unknown,
                                                    decay, state_noise_off,
                                                    state_noise_1st, bounds,
                                                    resolution, cov_offset,
                                                    a_coeff, r_coeff,
                                                    inv_offset, prior,
                                                    num_angles, detect_range)
    def sample_input_noise(self, u):
        return numpy.vstack((numpy.random.multivariate_normal(u[:2].reshape(-1),self.linQ).reshape((-1,1)), 
                             numpy.random.normal(u[2],self.nonlinQ))) 
    
    def cond_prob(self, eta, z, u):
        # calculate p(eta,z|self.eta, self.z, u)
        return None
    
    def get_R(self):
        return self.kf.R
    
    def get_lin_A(self):
        return self.kf.A
    
    def get_lin_B(self):
        return self.kf.B

    def get_lin_C(self):
        return self.kf.C

    def get_nonlin_B(self):
        # TODO
        #return numpy.zeros((1,2))
        return None
    
    def get_mix_B(self):
        # TODO
        # return numpy.vstack((self.get_nonlin_B(),self.kf.B))
        return None

    def get_nonlin_A(self):
        return numpy.zeros((1,2))
    
    def get_mix_A(self):
        # TODO
        # return numpy.vstack((self.get_nonlin_A(),self.get_lin_A()))
        return None
    
    def get_nonlin_Q(self):
        # TODO
        #return self.nonlinQ
        return None
    
    def get_lin_Q(self):
        # TODO
        #return self.linQ
        return None
    
    def get_mix_Q(self):
        # TODO
        #return self.mixQ
        return None
    
    def get_lin_est(self):
        return (self.kf.x_new, self.kf.P)

    def set_lin_state(self,inp):
        self.kf.x_new = inp
    
    def get_nonlin_state(self):
        # TODO 
        #return numpy.array([[self.c]])
        return None

    def set_nonlin_state(self,inp):
        # TODO
        #self.c = inp
        return None
    
    def set_lin_est(self,inp,Pin):
        self.kf.x_new = inp
        self.kf.P = Pin        