""" Module for working with ultrasonic distance sensors and occupancy grids """
import numpy as np
import math
import copy
import part_utils
import sonar
from occupancy_grid_fast import OccupancyGrid

class UltrasoundRobot(part_utils.ParticleSmoothingBaseRB):
    """ Robot using ultrasonic distance measurement for navigation """
    def __init__(self, offset, prec, rob, unknown, decay, bounds=None,
                 resolution=None, cov_offset = 1.0, a_coeff=1.0, r_coeff=1.0,
                 prior=0.5, num_angles=1, detect_range=np.Inf):

        self.robot = copy.deepcopy(rob)
        
        self.offset = np.asarray(offset)
        if (bounds == None):
            bounds = (-5.0, 5.0, -5.0, 5.0)
        if (resolution == None):
            resolution = 0.4
        self.map = OccupancyGrid(bounds=bounds, resolution=resolution,
                                 unknown=unknown, decay=decay, prior=prior,
                                 num_angles=num_angles)

        self.prec = prec
        self.cov_offset = cov_offset
        self.a_coeff = a_coeff
        self.r_coeff = r_coeff
        self.num_angles = num_angles
        self.detect_range = detect_range
        
    def update(self, u):
        """ Move robot according to kinematics """
        self.robot.update(u[:2])
        self.set_sensor_angle(u[2])
        self.map.time_update()

    def sample_input_noise(self, u):
        wn = np.copy(u)
        wn[:2,0] = self.robot.sample_input_noise(u[:2])
        return wn

    def calc_fov(self, dist):
        # Add offset of sensor, offset is specified in local coordinate system,
        # rotate to match global coordinates
        state = self.robot.get_state()
        theta = state[2]
        rot_mat = np.array(((math.cos(theta), -math.sin(theta)),
                    (math.sin(theta), math.cos(theta))))
        
        state[:2] = state[:2] + rot_mat.dot(self.offset[:2])
        
        # We can just add the angle, since unwrapping of
        # angles is handled inside FOVCone
        state[2] = state[2] + self.offset[2]
        
        cell_diag = 1.44*self.map.resolution
                    
        # Limit max distance of updates
        cone = sonar.FOVCone(state[:2], state[2], self.prec, min(dist+cell_diag, 15.0))
        # To improve performance we just extract which cells are within
        # the FOV once, and reause this both for measure and update.
        cells = self.map.find_visible_cells(cone)
        
        return cells
         
    def convert_measurement(self, cells, dist):
        """ Process new distance measurement """

        if (cells != None):
            # If outside "detect_range" don't detect anything, used update
            # the empty area in front

            # Create probability field for measurement
            sonar_measure = sonar.SonarMeasurement(cells, dist, self.map.resolution/2,
                                                   r_coeff=self.r_coeff,
                                                   a_coeff=self.a_coeff,
                                                   cov_offset=self.cov_offset,
                                                   num_angles=self.num_angles,
                                                   norm=False)
            
            # Eval measurement against map and update
            return sonar_measure
        
        else:
            return None
        

    def set_sensor_angle(self, theta):
        """ Move sensor to specified angle """
        self.offset[2] = theta

    def sample_smooth(self, filt_traj, ind, next_cpart):
        P = self.map.kf.P.ravel()
        z_est = self.map.kf.x_new.ravel()
        Q = self.map.kf.Q.ravel()
        
        
        H = P/(Q+P)
        Pi = P - H*P
        u = z_est + H*(next_cpart.z-z_est)
        
        cpart = self.collapse()
#        cpart.z = np.random.normal(u,Pi)
        cpart.z = u.ravel()
        
        return cpart

    def measure(self, dist):
        cells = self.calc_fov(dist)
        if (cells != None):
            prob = self.map.eval_measurement(cells, dist)
            sm = self.convert_measurement(cells, dist)
            if (sm):
                self.map(sm)
            return prob
        else:
            return 0.0
    
    def collapse(self):
        """ Return a sample of the particle where the linear states
        are drawn from the MVN that results from CLGSS structure """
        return UltrasoundRobotCollapsed(self)
    
    def clin_update(self, u):
        map = copy.deepcopy(self.map)
        map.time_update()
        return map
    
    def clin_measure(self, y):
        cells = self.calc_fov(y)
        sm = self.convert_measurement(cells, y)
        map = copy.deepcopy(self.map)
        if (sm):
            map(sm)
        return map
    
    def clin_smooth(self, z_next, u):
        map = copy.deepcopy(self.map)
        map.smooth(z_next)
        return map
    
    def linear_input(self, u):
        # Not used (in clin_update)
        return None 

    def set_nonlin_state(self, eta):
        self.robot.set_state(eta[0:7])
        self.set_sensor_angle(eta[7])
    
    def set_lin_est(self, lest):
        self.map = lest
        
    def get_lin_est(self):
        return self.map

class UltrasoundRobotCollapsed(object):
    """ Stores collapsed sample of MixedNLGaussian object """
    def __init__(self, parent):
        self.eta = np.hstack(((parent.robot.get_state()).ravel(),
                              parent.offset[2]))            
        self.z = np.copy(parent.map.kf.x_new.ravel())
