import robot
from ultrasound_fast import UltrasoundRobot
import PF
import PS
import math
import numpy
#from matplotlib.pyplot import *
import pygame
import sys
import copy
#import matplotlib.pyplot

class MyGUI(object):
    """ Draw a simple GUI for the SLAM visualisation """
    def __init__(self, ubot, use_gui):

        # Array dimensions
        self.dim = numpy.shape(ubot.map.get_map(0))
        self.dim = self.dim[::-1]
        self.num_angles = ubot.num_angles

        # Configure graphics
        if (use_gui):
            """ Create gui object """
            pygame.display.init()
            self.screen = pygame.display.set_mode( numpy.asarray((9, 4))*self.dim, 0, 32 )
        else:
            self.screen = pygame.Surface(numpy.asarray((8, 4))*self.dim, 0, 32)
        self.palette = tuple([(i, i, i) for i in range(256)])
        
        self.best_scale = numpy.array([4, 4])
        self.image_path = None
        self.use_gui = use_gui
        self.index = 1
        
    def set_image_path(self, path):
        self.image_path = path


    def save_image(self):
        filename = self.image_path + "%06d.jpg" % (self.index,)
        pygame.image.save(self.screen, filename)
        self.index += 1
    
    def draw_smoothing(self, straj_rev, best, particles, top_ind, dead_reckoning, m_slam):
        try:
            self.draw_best(best)
            self.plot_particles(best, particles, dead_reckoning, m_slam, top_ind)
            self.draw_top(particles[top_ind])
            
            scolor = (128, 128, 128, 0)
            pcolor = (255, 255, 0, 0)
            bounds = best.map.bounds
            
            for p in straj_rev:
                self.draw_part(p.eta[:2], bounds, pcolor, 1)
                
            if (self.use_gui):
                pygame.display.flip()
            if (self.image_path != None):
                self.save_image()
        except IndexError:
            print "GUI error!"
            
    
    def draw(self, best, particles, top_ind, dead_reckoning, m_slam):
        """ Draw everything """
#        tmp = 1000.0*map.get_map()
#
#        matplotlib.pyplot.pcolor(numpy.arange(map.bounds[0], map.bounds[1],
#                                                  map.resolution), 
#                                     numpy.arange(map.bounds[2], map.bounds[3],
#                                                  map.resolution),
#                                     tmp, shading='faceted')
#        matplotlib.pyplot.colorbar()
#        matplotlib.pyplot.show()
        #matplotlib.pyplot.show()
        try:
            self.draw_best(best)
            self.plot_particles(best, particles, dead_reckoning, m_slam, top_ind)
            self.draw_top(particles[top_ind])
            if (self.use_gui):
                pygame.display.flip()
            if (self.image_path != None):
                self.save_image()
        except IndexError:
            print "GUI error!"
            
    def draw_best(self, best):
        """ Draw the "best particle """
        self.draw_map(best.map, (0,0), self.best_scale)
    
    def draw_top(self, top):
        """ Draw the k-th best particle maps """
        if (len(top) >= 1):
            self.draw_map(top[0].map, self.dim*numpy.array((4, 0)), self.best_scale)
            if (self.use_gui):
                for k in range(1,min(len(top),5)):
                    self.draw_map(top[k].map, self.dim*numpy.array((8, (k-1)*self.num_angles)), (1, 1))
            
        return
    
    def draw_map(self, map, pos, scale):
        """ Draw map """     
   
        (prob, var) = map.get_nav_map()
        prob = numpy.copy(prob)
        var = numpy.copy(var)
#        lpmin = -20
#        lpmax = 20
#        lprob[lprob < lpmin] = lpmin
#        lprob[lprob > lpmax] = lpmax
#        lprob -= lpmin 
#        lprob *= (255.0/(lpmax-lpmin))
        comb = numpy.empty((prob.shape[1], prob.shape[0], 3))        


        # over-/underflow problems
        var[var < 0] = 0
        var[var > 50] = 50
        # arbitrary scaling to make it looks nice in the gui
        prob[var >= 50] = 0.0
        #prob[prob > 0.1] = 0.1
        #prob = 10.0*prob
        tmp = numpy.exp(-var.T*2.0)
        #tmp = var.T
        #tmp[tmp > 255.0] = 255.0
        tmp2 = 0.0*prob
        tmp2[var >= 50] = 255.0
        comb[:,:,0] = 255.0*prob.T
        comb[:,:,1] = 255.0*tmp
        comb[:,:,2] = tmp2.T
        #surf = pygame.surfarray.make_surface(lprob.T)
        surf = pygame.surfarray.make_surface(comb)
        surf = pygame.transform.scale(surf, numpy.asarray(scale)*numpy.asarray(surf.get_size()))
        #surf.set_palette(self.palette)
        surf = pygame.transform.flip(surf, False, True)
        self.screen.blit( surf, pos )
        pos = (pos[0], pos[1] + surf.get_size()[1])

    def draw_part(self,state, bounds, color, csize, offset=(0, 0)):
        """ Plot single particle """
        
        width = bounds[1] - bounds[0]
        height = bounds[3] - bounds[2]

        size = numpy.asarray((height, width))
        
        state = numpy.copy(state)
        state[1] = bounds[3] - state[1]
        state[0] -=  bounds[0]
        image_size = self.best_scale * self.dim[::-1]
        pos = state / size * image_size + self.best_scale / 2
        pos = numpy.array(pos, dtype=int)
        pygame.draw.circle(self.screen, color, (pos[0]+offset[0], pos[1]+offset[1]), csize, 0)
        
    def plot_particles(self, best, particles, dead_reckoning, m_slam, top_ind=None):
        """ Plot all particles along with best guess """
   
        best_color = (255, 255, 255, 0)
        nb1_color = (0, 0, 255, 0)
        black = (0, 0 ,0 ,0)
        white = (255, 255 ,255 ,0)
        grey = (128, 128 ,128 ,0)
        other_color = (0, 255, 255, 0)
        bad_color = (255, 20, 147, 0)
        bounds = best.map.bounds

        for p in particles:
            self.draw_part(p.robot.get_state()[:2], bounds, other_color, 1)
        if (top_ind != None):
            p = particles[top_ind[0]]
            self.draw_part(p.robot.get_state()[:2], bounds, nb1_color, 2)
            self.draw_part(p.robot.get_state()[:2], bounds, best_color, 2, self.dim*numpy.array((4, 0)))

        self.draw_part(m_slam[:2], bounds, grey, 4)
        self.draw_part(m_slam[:2], bounds, nb1_color, 3)
        self.draw_part(dead_reckoning.get_state()[:2], bounds, grey, 4)
        self.draw_part(dead_reckoning.get_state()[:2], bounds, bad_color, 3)
        #self.draw_part(best.robot.state[:2], bounds, grey, 3)
        self.draw_part(best.robot.get_state()[:2], bounds, best_color, 2)
        

def calc_est(slam):
    n_slam = len(slam.part)
    m_slam = numpy.zeros((3,))
    v_slam = numpy.zeros((3,))

    for i in range(n_slam):
        m_slam[:2] = m_slam[:2] + slam.w[i]*slam.part[i].robot.robot.state[:2]
        m_slam[2] = m_slam[2] + slam.w[i]*(numpy.mod(math.pi + slam.part[i].robot.robot.state[2], 
                                                      2*math.pi) - math.pi)
    m_slam = m_slam/sum(slam.w)
    for i in range(n_slam):
        v_slam[:2] = v_slam[:2] + slam.w[i]*(slam.part[i].robot.robot.state[:2]-m_slam[:2])**2
        v_slam[2] = v_slam[2] + slam.w[i]*(numpy.mod(math.pi + slam.part[i].robot.robot.state[2]-m_slam[2], 
                                                      2*math.pi) - math.pi)**2
    v_slam = v_slam/sum(slam.w)

    return (m_slam, v_slam)
    

def output_stats(out_file, ref, slam, dumb):
    # Calculate and output statistics for the different estimations
    
    (m_slam, v_slam) = calc_est(slam)
    
#    bias_slam = m_slam - ref.robot.state
#    bias_slam[2] = numpy.mod(math.pi + bias_slam[2], 2*math.pi) - math.pi
#    bias_dumb = m_dumb - ref.robot.state
#    bias_dumb[2] = numpy.mod(math.pi + bias_dumb[2], 2*math.pi) - math.pi
#    
    best_ind = numpy.argsort(slam.w)[0]
#    best_bias = numpy.zeros((3,))
#    best_bias[:2] = slam.part[best_ind].robot.state[:2] - ref.robot.state[:2]
#    best_bias[2] = numpy.mod(math.pi + slam.part[best_ind].robot.state[2] - ref.robot.state[2], 
#                             2*math.pi) - math.pi
                             
    # Ref (x,y,theta) dead-reckoning (x,y,theta) slam_avg (x,y,theta) slam_var (x,y,theta), slam_best (x,y,theta)
    
    line  = "%f, %f, %f,    " % tuple(ref.robot.robot.state)
    line += "%f, %f, %f,   " % tuple(dumb.state)
    line += "%f, %f, %f,   " % tuple(m_slam)
    line += "%f, %f, %f,   " % tuple(v_slam)
    line += "%f, %f, %f\n"   % tuple(slam.part[best_ind].robot.robot.state)

    
    out_file.write(line)


old_angle = math.pi/2

#numpy.seterr(all='raise')

config_phase = True
# Place-holder, will be assigned after config is determined
rob = None
ubot = None
pf = None
pf_dumb = None
gui = None

# Default values match Lego Mindstorms robot
wheel_base = 140.0/1000.0
wheel_size = 80.0/1000.0
wheel_ticks = -360.0
beam_width = math.pi/5
num_part = 50
angle_ticks = -360*5 # gear ratio 1:5, positive clockwise
dist_scale = 1.0/100.0 # cm to m
enc_noise = 0.1
enc_noise_lin = 0.1
theta_noise = 0.1
theta_noise_lin = 0.1
#noise = numpy.asarray([2, 2, 0.05])
#state_noise_off = numpy.asarray([0.05, 0.05, 0.01])
#state_noise_1st = numpy.asarray([0.0, 0.0, 0.0])
bounds = (-2.0, 2.0, -2.0, 2.0)
offset = (0.09, 0.0, 0.0)
resolution = 0.05
image_path = None
stats_file = None
trunc_readings = None
lp_coeff = None
a_coeff = 5.0
r_coeff = 1.0
cov_offset = 1.0
#inv_offset = 1.0
decay = 0.0001
unknown = 50
num_angles = 1
detect_range = numpy.Inf
prior = 0.1
use_gui = True
# Smoothing config
num_back_traj = 10
filter_steps = 20
overlap_steps = 10
smooth_threshold = 2.0/3.0
min_smooth_len = 2
nth_smoothing = -1
resample_count = 0
pf_resample = 2.0/3.0
filter_only = False

no_more_data = False

j = 1
while not no_more_data:
    line = sys.stdin.readline()
    sys.stdout.write(line)
    if line == '':
        no_more_data = True
    if line == '\n':
        continue
    words = line.split()
    
    if (config_phase):
        if (words[0].lower() == "wheel_base:".lower()):
            wheel_base = float(words[1])
        if (words[0].lower() == "wheel_size:".lower()):
            wheel_size = float(words[1])
        if (words[0].lower() == "wheel_ticks:".lower()):
            wheel_ticks = float(words[1])
        if (words[0].lower() == "beam_width:".lower()):
            beam_width = float(words[1])
        if (words[0].lower() == "num_part:".lower()):
            num_part = int(float(words[1]))
        if (words[0].lower() == "num_angles:".lower()):
            num_angles = int(float(words[1]))
        if (words[0].lower() == "lp_coeff:".lower()):
            lp_coeff = float(words[1])
        if (words[0].lower() == "a_coeff:".lower()):
            a_coeff = float(words[1])
        if (words[0].lower() == "r_coeff:".lower()):
            r_coeff = float(words[1])
        if (words[0].lower() == "cov_offset:".lower()):
            cov_offset = float(words[1])
#        if (words[0].lower() == "inv_offset:".lower()):
#            inv_offset = float(words[1])            
        if (words[0].lower() == "angle_ticks:".lower()):
            angle_ticks = float(words[1])
        if (words[0].lower() == "unknown:".lower()):
            unknown = float(words[1])
        if (words[0].lower() == "decay:".lower()):
            decay = float(words[1])
        if (words[0].lower() == "prior:".lower()):
            prior = float(words[1])            
        if (words[0].lower() == "dist_scale:".lower()):
            dist_scale = float(words[1])
        if (words[0].lower() == "enc_noise:".lower()):
            enc_noise = numpy.asarray([ float(words[1]),])
        if (words[0].lower() == "enc_noise_lin:".lower()):
            enc_noise_lin = numpy.asarray([ float(words[1]),])
        if (words[0].lower() == "theta_noise:".lower()):
            theta_noise = numpy.asarray([ float(words[1]),])
        if (words[0].lower() == "theta_noise_lin:".lower()):
            theta_noise_lin = numpy.asarray([ float(words[1]),])
#        if (words[0].lower() == "noise:".lower()):
#            noise = numpy.asarray([ float(words[1]), float(words[2]), float(words[3])])
#        if (words[0].lower() == "state_noise_off:".lower()):
#            state_noise_off = numpy.asarray([ float(words[1]),
#                                              float(words[2]),
#                                              float(words[3])])
#        if (words[0].lower() == "state_noise_1st:".lower()):
#            state_noise_1st = numpy.asarray([ float(words[1]),
#                                              float(words[2]),
#                                              float(words[3])])
        if (words[0].lower() == "offset:".lower()):
            offset = [ float(words[1]), float(words[2]), float(words[3])]
        if (words[0].lower() == "bounds:".lower()):
            bounds = ( float(words[1]), float(words[2]),
                       float(words[3]), float(words[4]))
        if (words[0].lower() == "resolution:".lower()):
            resolution = float(words[1])
        if (words[0].lower() == "trunc_readings:".lower()):
            trunc_readings = float(words[1])
        if (words[0].lower() == "detect_range:".lower()):
            detect_range = float(words[1])
        if (words[0].lower() == "save_images:".lower()):
            image_path = words[1]
        if (words[0].lower() == "stats_file:".lower()):
            stats_file = open(words[1], 'w', 1024*1024)
        if (words[0].lower() == "disable_gui".lower()):
            use_gui = False
        if (words[0].lower() == "num_back_traj:".lower()):
            num_back_traj = int(words[1])
        if (words[0].lower() == "filter_steps:".lower()):
            filter_steps = int(words[1])
        if (words[0].lower() == "overlap_steps:".lower()):
            overlap_steps = int(words[1])
        if (words[0].lower() == "smooth_threshold:".lower()):
            smooth_threshold = float(words[1])
        if (words[0].lower() == "min_smooth_len:".lower()):
            min_smooth_len = int(words[1])
        if (words[0].lower() == "nth_smoothing:".lower()):
            nth_smoothing = int(words[1])
        if (words[0].lower() == "pf_resample:".lower()):
            pf_resample = float(words[1])
        if (words[0].lower() == "filter_only:".lower()):
            tmp = words[1].lower()
            if (tmp in ['true', '1', 'on', 'yes']):
                filter_only = True
            else:
                filter_only = False
        if (words[0].lower() == "measure:".lower() or
            words[0].lower() == "update:".lower() or
            words[0].lower() == "refpos:".lower()):
        
            config_phase = False
        
            print "Using config: "
            print "\twheel_base: %f" % wheel_base
            print "\twheel_size: %f" % wheel_size
            print "\twheel_ticks: %f" % wheel_ticks
            print "\tbeam_width: %f" % beam_width
            print "\tnum_part: %f" % num_part
            print "\tangle_ticks: %f" % angle_ticks
            print "\tdist_scale: %f" % dist_scale
#            print "\tnoise: %f %f %f" % tuple(noise)
            print "\tenc_noise: %f" % enc_noise
            print "\tenc_noise_lin: %f" % enc_noise_lin
            print "\ttheta_noise: %f" % theta_noise
            print "\ttheat_noise_lin: %f" % theta_noise_lin
#            print "\tstate_noise_off: %f %f %f" % tuple(state_noise_off)
#            print "\tstate_noise_1st: %f %f %f" % tuple(state_noise_1st)
            print "\toffset: %f %f %f" % tuple(offset)
            print "\tbounds: %f %f %f %f" % bounds
            print "\tresolution: %f" % resolution
            if (image_path):
                print "\timage path: %s" % image_path
            if (trunc_readings):
                print "\ttrunc_readings: %f" % trunc_readings
            if (use_gui):
                print "\tGUI enabled"
            else:
                print "\tGUI disabled"
            print "\tdetect_range: %f" % detect_range
            print "\tlp_coeff: %s" % lp_coeff
            print "\ta_coeff: %f" % a_coeff
            print "\tr_coeff: %f" % r_coeff
            print "\tcov_offset: %f" % cov_offset
#            print "\tinv_offset: %f" % inv_offset
            print "\tunknown: %f" % unknown
            print "\tdecay: %f" % decay
            print "\tprior: %f" % prior
            print "\tnum_angles: %f" % num_angles
            print "\tstats_file: %s" % str(stats_file)
            print "\tnum_back_traj: %f" % num_back_traj
            print "\tfilter_steps: %f" % filter_steps
            print "\toverlap_steps: %f" % overlap_steps
            print "\tpf_resample: %s" % str(pf_resample) 
            print "\tfilter_only: %s" % str(filter_only)
            print "\tsmooth_threshold: %f" % smooth_threshold
            print "\tmin_smooth_len: %d" % min_smooth_len
            print "\tnth_smoothing: %d" % nth_smoothing
            # Configure all variables/objects
            dead_reckoning = robot.ExactDifferentialRobot(
                                          l=wheel_base,d=wheel_size,
                                          ticks=wheel_ticks,
                                          state=(0.0, 0.0, old_angle))
            rob = robot.DifferentialRobot(l=wheel_base,d=wheel_size,
                                          ticks=wheel_ticks,
                                          enc_noise=enc_noise,
                                          enc_noise_lin=enc_noise_lin,
                                          theta_noise=theta_noise,
                                          theta_noise_lin=theta_noise_lin,
                                          state=(0.0, 0.0, old_angle))
            ubot = UltrasoundRobot(offset=offset,
                                   prec=beam_width,
                                   rob=rob,
                                   bounds=bounds,
                                   resolution=resolution,
                                   cov_offset=cov_offset,
                                   a_coeff=a_coeff,
                                   r_coeff=r_coeff,
                                   unknown=unknown,
                                   decay=decay,
                                   prior=prior,
                                   num_angles=num_angles,
                                   detect_range=detect_range)
            
            pa = PF.ParticleApproximation(num=num_part, seed=ubot)
            pt = PF.ParticleTrajectory(pa, resample=pf_resample, lp_hack=lp_coeff)
            
            gui = MyGUI(ubot, use_gui)
            if (image_path):
                gui.set_image_path(image_path)
            
    if (not config_phase):
        if ((not no_more_data) and words[0].lower() == "refpos:".lower()):
            # Update correct position of robot in global referensframe
            x = float(words[1])
            y = float(words[2])
            theta = float(words[3])
            ubot.robot.set_pos(x,y,theta)
            # Ref position comes before measurements update, calculate rel. err after
            # receiving measurement.
        else:     
            if ((not no_more_data) and words[0].lower() == "measure:".lower()):
                # DifferentialRobot will handle conversion of these measurements
                wa = float(words[1])
                wb = float(words[2])
                # Convert to angle
                a = float(words[3])/angle_ticks*2*math.pi
                # Convert measurment scales
                d = float(words[4])*dist_scale 
                
                ubot.update([0.0, 0.0, a, 0.0])
                dead_reckoning.kinematic([wa, wb])
                pt.update([wa, wb, a])
                if ((not trunc_readings) or (d < trunc_readings)):
                    ubot.measure(d)
                    resampled = pt.measure(d)
                    if (resampled):
                        resample_count += 1
                    
                # We need to store this value since "update" messages don't contain it
                old_angle = a
    
                if (stats_file != None):
                    output_stats(stats_file, ubot, pt[-1].pa, dead_reckoning)
                
            if ((not no_more_data) and words[0].lower() == "update:".lower()):
                wa = float(words[1])
                wb = float(words[2])
                ubot.update([0.0, 0.0, old_angle, 0.0])
                dead_reckoning.kinematic([wa, wb])
                
                # update message doesn't contain angle, use old value
                pt.update([wa, wb, old_angle])
                
                if (stats_file != None):
                    output_stats(stats_file, ubot, pt[-1].pa, dead_reckoning)
    
            best_ind = pt[-1].pa.find_best_particles(min(5, num_part))
            #gui.draw(pf.particles[best_ind[0]], pf.particles, best_ind[1:])
            if (use_gui or image_path):
                (m_slam, v_slam) = calc_est(pt[-1].pa)
                gui.draw(ubot, pt[-1].pa.part, best_ind, dead_reckoning, m_slam)
            
            # Do smoothing when particle efficency is too low or after fixed nbr of steps
            if ((not filter_only) and len(pt) > min_smooth_len and 
                (pt.efficiency() < smooth_threshold or 
                 len(pt) == filter_steps or no_more_data or 
                 (nth_smoothing > 0 and resample_count >= nth_smoothing))):
    
                resample_count = 0
                smooth_ind = max(0, len(pt) - overlap_steps, int(math.ceil(len(pt)/2.0)))
                signals = pt.extract_signals()
                print "smoothing: len(pt)=%d, smooth_ind=%d, eff=%f" % (len(pt), smooth_ind,pt.efficiency())
                
                # Used in CB below
                (m_slam, v_slam) = calc_est(pt[-1].pa)
                best_ind = pt[-1].pa.find_best_particles(min(5, num_part))
                def draw_smoothing_cb(st_rev):
                    gui.draw_smoothing(st_rev, ubot, pt[-1].pa.part, best_ind,
                                      dead_reckoning, m_slam)
                    
                #st = PS.do_smoothing(pt,num_back_traj, callback=draw_smoothing_cb)
                st = PS.do_smoothing(pt,num_back_traj, callback=None)
                t0 = st[0].t[smooth_ind]
                pt = None # Release references to no longer needed memory
                st = PS.do_rb_smoothing(st)
                pa = PS.extract_smooth_approx(st, smooth_ind)
                st = None # Release references to no longer needed memory
                # Resample to keep number of filtering particles constant
                pa.resample(num_part)
                # Create new ParticleTrajectory using smoothed approximation as initialisation
                pt = PF.ParticleTrajectory(pa, resample=pf_resample, t0=t0, lp_hack=lp_coeff)

                def draw_replay_cb(pt):
                    (m_slam, v_slam) = calc_est(pt[-1].pa)
                    best_ind = pt[-1].pa.find_best_particles(min(5, num_part))
                    gui.draw(ubot, pt[-1].pa.part, best_ind, dead_reckoning, m_slam)
                
                # Propagate particle using old input and measurements 
                #PS.replay(pt, signals, smooth_ind, callback=draw_replay_cb)
                PS.replay(pt, signals, smooth_ind, callback=None)
                # Reset dumb trajectory to conserv memory
            if (filter_only):
                pt = pt.spawn()
            
        j += 1
    

print "Simulation done"
 

    

