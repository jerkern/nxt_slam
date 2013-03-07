""" Module for working with ultrasonic distance sensors and occupancy grids """
import numpy as np
import math
import copy
import scipy.stats
import part_utils
import pdfutils
import sonar
from occupancy_grid_fast import OccupancyGrid

class UltrasoundRobot(part_utils.ParticleSmoothingBaseRB):
    """ Robot using ultrasonic distance measurement for navigation """
    def __init__(self, offset, prec, rob, enc_noise, enc_noise_lin,
                 theta_noise, theta_noise_lin, 
                 unknown, decay, bounds=None,
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
        self.enc_noise_lin = enc_noise_lin
        self.motion = (0.0, 0.0)

        self.theta_noise = theta_noise
        self.theta_noise_lin = theta_noise_lin
        
        self.cur_enc_noise = enc_noise
        self.cur_theta_noise = theta_noise

    def update(self, data):
        """ Move robot according to kinematics """
        motion = self.robot.kinematic(data[:2])
        self.cur_enc_noise = self.enc_noise + np.abs(motion[0])*self.enc_noise_lin
        self.cur_theta_noise = self.theta_noise + np.abs(motion[1])*self.theta_noise_lin
        
        self.robot.add_state_noise((0, 0, data[3]))
        self.set_sensor_angle(data[2])
        self.map.time_update()
        return motion
        
 
    def sample_input_noise(self, w):
        
        w = np.asarray(w)
        
        wn = np.random.uniform(w[:2,:]-self.cur_enc_noise/2.0, 
                               w[:2,:]+self.cur_enc_noise/2.0, (2,1))
        
        theta_noise = np.random.normal(0,self.cur_theta_noise)
        
        return np.hstack((wn.ravel(), w[2,:], theta_noise))
            
    def calc_fov(self, dist):
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


    def calc_pdfs(self, u, theta_n=None):
        
        # Calc prob. of next theta
        if (theta_n == None):
            theta_n = self.robot.calc_next_theta(u[:2])
        
        enc_noise = self.enc_noise + np.abs(self.motion[0])*self.enc_noise_lin
        
        a = np.asarray((-0.5, 0.5))*enc_noise/(2.0*self.robot.l)
        #b = np.asarray((-0.5, 0.5))*self.enc_noise/(-2.0*self.robot.l)
        b = -a
        
        theta_pdf = pdfutils.unifsum(a,b)
       
        # Calc prob of x and y given theta
        next_xy = self.robot.calc_next_xy(u[:2], theta_n)
        tmp = np.asarray((-0.5, 0.5))*enc_noise*self.robot.C/4.0
        a = tmp*math.cos(theta_n)
        #b = np.asarray((-0.5, 0.5))*self.enc_noise*self.robot.C/4.0*cmath.cos(theta_n)
        x_pdf = pdfutils.unifsum(a,a)

        a = tmp*math.sin(theta_n)
        #b = np.asarray((-0.5, 0.5))*self.enc_noise*self.robot.C/4.0*math.sin(theta_n)
        
        y_pdf = pdfutils.unifsum(a,a)
        
        return (theta_pdf, x_pdf, y_pdf)


    def next_pdf(self, next, u):
        theta_n = next.eta[2]
        x_n = next.eta[0]
        y_n = next.eta[1]
        
        theta = self.robot.state[2]
        x = self.robot.state[0]
        y = self.robot.state[1]

        dx = x_n - x
        dy = y_n - y

        tmp1 = (4.0/self.robot.C*dy-(self.robot.old_S-self.robot.old2_S)*math.sin(theta))
        tmp2 = (4.0/self.robot.C*dx-(self.robot.old_S-self.robot.old2_S)*math.cos(theta))

        # Calculate sum of inputs for reaching new x and y coords, there are two solutions!
        K = math.sqrt((tmp1)**2 + (tmp2)**2)
       
        S1 = K + self.robot.old_S
        S2 = -K + self.robot.old_S
         
        # Calculate new angle for reaching provide x and y coords, thera are two solutions
        theta_hat1 = math.atan2(tmp1, tmp2)
        #theta_hat2 = math.atan2(-tmp1, -tmp2)
        theta_hat2 = theta_hat1 + math.pi
        # Calculate diff of inputs needed for reaching new angle, always assume the robot hasn't turned
        # more than halt a revolution
        theta_diff1 = np.mod(theta_hat1-theta+math.pi, 2.0*math.pi) - math.pi
        theta_diff2 = np.mod(theta_hat2-theta+math.pi, 2.0*math.pi) - math.pi
        D1 = 2.0/self.robot.C*self.robot.l*theta_diff1+self.robot.old2_D
        D2 = 2.0/self.robot.C*self.robot.l*theta_diff2+self.robot.old2_D
        
        # Calcuate needed inputs
        Pr1 = 0.5*(S1+D1)
        Pl1 = 0.5*(S1-D1)
        
        Pr2 = 0.5*(S2+D2)
        Pl2 = 0.5*(S2-D2)
       
        # Check probability of inputs
        prob_Pr1 = 1.0/self.cur_enc_noise if (Pr1 >= u[1]-self.cur_enc_noise/2.0 and Pr1 <= u[1]+self.cur_enc_noise/2.0) else 0.0
        prob_Pl1 = 1.0/self.cur_enc_noise if (Pl1 >= u[0]-self.cur_enc_noise/2.0 and Pl1 <= u[0]+self.cur_enc_noise/2.0) else 0.0
        prob_Pr2 = 1.0/self.cur_enc_noise if (Pr2 >= u[1]-self.cur_enc_noise/2.0 and Pr2 <= u[1]+self.cur_enc_noise/2.0) else 0.0
        prob_Pl2 = 1.0/self.cur_enc_noise if (Pl2 >= u[0]-self.cur_enc_noise/2.0 and Pl2 <= u[0]+self.cur_enc_noise/2.0) else 0.0
        
        try:
            (prob_theta1, prob_theta2) = scipy.stats.norm.pdf((theta_hat1, theta_hat2), loc=theta_n, shape=self.cur_theta_noise)
        except FloatingPointError:
            prob_theta1 = 0.0
            prob_theta2 = 0.0
            
        return max(prob_Pr1*prob_Pl1*prob_theta1,prob_Pr2*prob_Pl2*prob_theta2)
    
    def fwd_peak_density(self, u):
        return ((1.0/self.cur_enc_noise)**2 *
                scipy.stats.norm.pdf(0, loc=0, shape=self.cur_theta_noise))
        
        
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
        self.robot.state[:] = eta[0:3]
        self.robot.old_S = eta[4]
        self.robot.old2_S = eta[5]
        self.robot.old_D = eta[6]
        self.robot.old2_D = eta[7]
        self.set_sensor_angle(eta[3])
    
    def set_lin_est(self, lest):
        self.map = lest
        
    def get_lin_est(self):
        return self.map

class UltrasoundRobotCollapsed(object):
    """ Stores collapsed sample of MixedNLGaussian object """
    def __init__(self, parent):
#        self.eta = np.copy(parent.robot.state).ravel()            
#        lin_est = parent.map.kf.x_new.ravel()
#        lin_P = parent.map.kf.P.ravel()
#        tmpp = np.random.normal(lin_est,lin_P)
#        self.z = tmpp.ravel()
        self.eta = np.hstack(((parent.robot.state).ravel(), parent.offset[2],
                              parent.robot.old_S, parent.robot.old2_S,
                              parent.robot.old_D, parent.robot.old2_D))            
        self.z = np.copy(parent.map.kf.x_new.ravel())
