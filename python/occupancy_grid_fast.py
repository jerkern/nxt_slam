""" Module for working with occupancy grids """
from kalman_fast import KalmanSmootherFast
import numpy
import math
#import scipy.sparse as sp

class OccupancyGrid(object):
    """ Occupancy grid type map """
    def __init__(self, bounds=(-10.0, 10.0, -10.0, 10.0), resolution=0.2,
                 prior=0.5, unknown=50.0, decay=0.00001, num_angles=1):
        """ bounds: the extends of the map, in m
        resolution: the size of each grid cell, in m """

        assert(bounds[1] > bounds[0])
        assert(bounds[3] > bounds[2])
        assert(resolution > 0)

        numx = int(math.ceil((bounds[1] - bounds[0])/resolution))
        numy = int(math.ceil((bounds[3] - bounds[2])/resolution))
        
        tot_prod = int(numx*numy*num_angles)
        
        self.shape = (numy, numx, num_angles)
        self.bounds = bounds
        self.resolution = resolution
        big_covar = unknown #1000.0
        small_covar = decay #10^-5
        p_occ = logodds(prior)
        # C, R matrix will be changed for every measurement, these
        # are just place holder values

        x0 = p_occ*numpy.ones((tot_prod,1))
        P0 = big_covar*numpy.ones((tot_prod,1))
        Q = small_covar*numpy.ones((tot_prod,1))
        R = numpy.ones((tot_prod,1))
        self.kf = KalmanSmootherFast(x0=x0, P0=P0, Q=Q, R=R)

    def __call__(self, sonar_measurement):
        indices = sonar_measurement.indices
        # Convert probabilities to log odds format first
        prob = logodds(sonar_measurement.prob)
        prob = prob[:, numpy.newaxis]
        n = sonar_measurement.var.shape[0]
        R = numpy.reshape(numpy.asarray(sonar_measurement.var),(n,1))
        ind = indices[:, 2]*self.shape[1]*self.shape[0] + indices[:, 1]*self.shape[0]+indices[:, 0]

        return self.kf.meas_update(prob, ind, R)
    
    def eval_measurement(self, cells, dist):
        """ Evaluate measurements against existing map """
        
        def ownpdf(x, m, sigma):
            """ Non-normalized pdf for cell/sensor measurement 
            x - eval points
            m - mean, either scaler or same shape as x
            sigma - 'width' of distribution """

            pdf = numpy.zeros(numpy.shape(x))

            top_ind = (abs(x-m) <= sigma/2.0)
            if (top_ind.any()):
                pdf[top_ind] = 1.0
            interp_ind = (abs(x-m) > sigma/2.0) & (abs(x-m) < sigma)
            if (interp_ind.any()):
                pdf[interp_ind] = (
                       1.0 - (abs(x-m)[interp_ind]-sigma/2)/(sigma/2)
                       ) 
            return pdf
        
        num_angles = self.shape[2]
        cells.sort(order='r', axis=0)
        tmp = (cells['a_w'] + math.pi)*num_angles/(2.0*math.pi)
        tmp = numpy.floor(tmp)
        # Wrap-around
        tmp[tmp >= num_angles] = num_angles-1
        tmp = numpy.reshape(tmp,(-1,1))
        indices = numpy.concatenate((cells['opt'], tmp), 1)
        ind = indices[:, 2]*self.shape[1]*self.shape[0] + indices[:, 1]*self.shape[0]+indices[:, 0]
        ind = ind.astype(int)
        pdf = ownpdf(cells['r'], dist, self.resolution)
        prob_occ = inv_logodds(self.kf.x_new[ind]).ravel()
        
        #multiplier = 1.0
        #prob = 0
        #for j in range(numpy.shape(cells)[0]):
        #    prob += multiplier*prob_occ[j]*pdf[j]
        #    multiplier *= (1 - prob_occ[j])
        
        # Same calculations as for loop above, but much more efficient
        multiplier = numpy.cumprod(1 - prob_occ)
        multiplier = numpy.roll(multiplier, 1)
        multiplier[0] = 1.0
        prob = numpy.dot(multiplier, prob_occ*pdf)
        return prob
    
    def time_update(self):
        self.kf.time_update()
        
    def smooth(self, z_next):
        self.kf.smooth(z_next.kf.x_new, z_next.kf.P)
    
#    def calc_C(self, indices):
#        nbr_y = numpy.size(indices,0)
##        C = numpy.zeros((nbr_y, numpy.prod(self.shape)))
##        for i in range(nbr_y):
##            # Convert to linear indexing
##            ind = indices[i, 1]*self.shape[0]+indices[i, 0]
##            C_old[i,ind] = 1
#        y = range(nbr_y)
#        x = indices[:, 2]*self.shape[1]*self.shape[0] + indices[:, 1]*self.shape[0]+indices[:, 0]
#        C_data = ((1,)*nbr_y, (y, x))
#        C_shape = (nbr_y, numpy.prod(self.shape))
#        C = sp.coo_matrix(C_data, shape=C_shape)
#        return C.tocsr()

    def get_map(self, axis=None):
        # Return the mean of the estimates for each angle,
        # Needs to convert to array first, matrix types seem
        # to loose the single dimension during reshape
        tmp = numpy.reshape(numpy.asarray(self.kf.x_new),
                            self.shape, order='F')
        #return inv_logodds(numpy.mean(tmp,2))
        if (axis != None):
            return inv_logodds(tmp[:,:,axis])
        return inv_logodds(numpy.max(tmp,2))
    
    def get_nav_map(self):
        
        
        tmp_prob = numpy.reshape(numpy.asarray(self.kf.x_new),
                                 (-1,self.shape[2]), order='C')
        tmp_var = numpy.reshape(numpy.asarray(self.kf.P),
                                (-1,self.shape[2]), order='C')
        
        ind = numpy.argmax(tmp_prob,axis=1)
        
        tmp_prob = tmp_prob[numpy.arange(tmp_prob.shape[0]), ind]
        tmp_var = tmp_var[numpy.arange(tmp_var.shape[0]), ind]
        
        tmp_prob = tmp_prob.reshape((self.shape[0],self.shape[1]))
        tmp_var = tmp_var.reshape((self.shape[0],self.shape[1]))
        
        #ind = numpy.argmax(tmp_prob,axis=2)
        return (inv_logodds(tmp_prob), tmp_var)
    
    def get_variance(self, axis=None):
        tmp = self.kf.P
        #tmp2 = sp.spdiags(tmp,0,tmp.size,tmp.size).tocsr()
        tmp = numpy.reshape(numpy.asarray(tmp), self.shape, order='F')
        if (axis != None):
            return tmp[:,:,axis]
        return numpy.mean(tmp,2)
        #return numpy.min(tmp,2)

    def find_visible_cells(self, cone):
        """ Finds all cells centers of all grids within the cone specified
        state - (x,y,dir)
        prec - FOV width, summetric around dir
        dist - max distance to search """
        
        def course_crop(dim, bounds, cone):
            """ Rectangular bounds on which indices need to be evaluated """
            (xmin, xmax, ymin, ymax) = cone.get_bounding_rect()

            xleft = math.floor((dim[1]-1)*
                               (xmin-bounds[0])/(bounds[1]-bounds[0]))
            xright = math.ceil((dim[1]-1)*
                               (xmax-bounds[0])/(bounds[1]-bounds[0]))
            
            xleft = max((xleft, 1))
            xright = min((xright, dim[1]-1))
    
            yupp = math.floor((dim[0]-1)*
                              (ymin-bounds[2])/(bounds[3]-bounds[2]))
            ylow = math.ceil((dim[0]-1)*
                             (ymax-bounds[2])/(bounds[3]-bounds[2]))
            
            yupp = max((yupp, 1))
            ylow = min((ylow, dim[0]-1))

            return (int(xleft), int(xright), int(yupp), int(ylow))

        
        (xle, xri, yup, ylo) = course_crop(self.shape, self.bounds, cone)

        # Evaluate each cell center for the FOV
        x_ind = numpy.arange(xle, xri+1, dtype=numpy.int)
        y_ind = numpy.arange(yup, ylo+1, dtype=numpy.int)
        
        x, y = numpy.meshgrid(x_ind*self.resolution+self.bounds[0],
                              y_ind*self.resolution+self.bounds[2])

        x_ind, y_ind = numpy.meshgrid(x_ind, y_ind)
        
        x = x.reshape((-1, 1))
        y = y.reshape((-1, 1))
        x_ind = x_ind.reshape((-1, 1))
        y_ind = y_ind.reshape((-1, 1))
        
        if (x.shape[0] == 0):
            return None
            
        # cells will contains (a,r,y_ind,x_ind) for all visible cells
        cells  = cone(y[:, 0], x[:, 0], numpy.hstack([y_ind, x_ind]))
        
        if (cells == None):
            return None

        cells.sort(order='r', axis=0)

        return cells

def inv_logodds(lodds):
    """ Convert logodds data to probability """
    lodds[lodds > 50] = 50
    lodds[lodds < -50] = -50
    odds = numpy.exp(lodds)
    return odds / (1.0 + odds)


def logodds(prob):
    """ Convert probability to logodds format """
    return numpy.log(prob / (1.0 - prob))
