import numpy as np
import math

class FOVCone(object):
    """ Represent a FOV measurment """
    def __init__(self, pos, theta, prec, dist):
        self.xpos = pos[0]
        self.ypos = pos[1]
        self.theta = theta
        self.prec = prec
        self.dist = dist


    def __call__(self, y, x, optdata = None):
        """ Check if inside FOV, if so return coordinate and angle distance
        for each y,x pair. If optdata is provided then the corresponding
        entries are also returned.
        
        y,x (and opdata) must have the same length """

        x = np.asarray(x)
        y = np.asarray(y)
        # Absolute angle, world coordinates
        a_w = np.arctan2(y-self.ypos, x-self.xpos)
        
        # Angle relative sensor, use mod to enforce interval [-pi,pi]
        tmp = a_w - (self.theta - math.pi)
        a = np.mod(tmp, 2*math.pi) - math.pi 
        
        ind = np.abs(a) <= self.prec/2.0
        
        if (not ind.any()):
            return None
        
        # Throw away all pairs outside FOV
        newx = x[ind]-self.xpos
        newy = y[ind]-self.ypos 
        newa = a[ind]
        newa_w = a_w[ind]
        
        # Calculating this manually instead of using norm
        # and apply_along_axis is _much_ faster
        r = np.sqrt((newx ** 2) + (newy ** 2))
        
       
        rind = (r <= self.dist)

        cnt = newa[rind].shape[0]
        
        if (cnt == 0):
            return None


        # Create structure array contanings cells within FOV
        if (optdata != None):
            rval = np.empty(cnt, dtype=[('a', float),
                                        ('r', float),
                                        ('a_w', float),
                                        ('opt', optdata.dtype,
                                         optdata.shape[1])])
            rval['opt'] = (optdata[ind, :])[rind, :]
        else:
            rval = np.empty([cnt, 2], dtype=[('a', float),
                                             ('r', float),
                                             ('a_w', float)])
        
        rval['a'] = newa[rind]
        rval['r'] = r[rind]
        rval['a_w'] = newa_w[rind]
        
        return rval
        
    
    def get_bounding_rect(self):
        """ Find bounding rectange of FOV cone """
        tmp1 = self.xpos + self.dist*math.cos(self.theta+self.prec/2.0)
        tmp2 = self.xpos + self.dist*math.cos(self.theta-self.prec/2.0)
        xmin = np.min((self.xpos, tmp1, tmp2))
        xmax = np.max((self.xpos, tmp1, tmp2))
            
        tmp1 = self.ypos + self.dist * math.sin(self.theta + self.prec / 2.0)
        tmp2 = self.ypos + self.dist * math.sin(self.theta - self.prec / 2.0)
        ymin = min((self.ypos, tmp1, tmp2))
        ymax = max((self.ypos, tmp1, tmp2))

        return (xmin, xmax, ymin, ymax)

            
class SonarMeasurement(object):
    """ Class for handling sonar measurements,
    converts from range to probability field """
    def own_pdf(self, r, mu, scale):
        pdf = 0.0*r
        ind1 = np.abs(r-mu) < scale / 2
        ind2 = np.abs(r-mu) < scale

        max = 0.99
        min = 0.01
        pdf[ind1] = max
        interpolate_inds = (~ind1)*ind2
        pdf[interpolate_inds] = max* (1 - 2*(np.abs(r[interpolate_inds] - mu) - scale/2)/scale)
        pdf[pdf < min] = min
        
        return pdf

    def __init__(self, cells, dist, prec, r_coeff, a_coeff, cov_offset, num_angles, norm=True):
        """ Create probabilty field evaluated in 'cells' for distance 'dist' """

        tmp = (cells['a_w'] + math.pi)*num_angles/(2.0*math.pi)
        tmp = np.floor(tmp)
        # Wrap-around
        tmp[tmp >= num_angles] = num_angles-1
        tmp = np.reshape(tmp,(-1,1))
        self.indices = np.concatenate((cells['opt'], tmp), 1)
        r = cells['r']
        a = cells['a']
        self.prob = self.own_pdf(r, dist, prec)
        
        # Probabilty must sum to one
        # However, there is a small chance of a spurios measurment, also numerically
        # we run into trouble if any single cell would get p=1.0
        if (norm):
            total = np.sum(self.prob)
            if (total > 0.0):
                self.prob = 0.99*self.prob / total
            else:
                self.prob = self.prob + 0.01
            
        self.var = r_coeff*r + a_coeff*abs(a) + cov_offset
        self.var[r > (dist+prec/2.0)] = 1000
