""" Module for working with occupancy grids """
from kalman import KalmanFilter
import numpy
import math
import scipy.sparse as sp

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
        A = (sp.eye(tot_prod, tot_prod)).tocsr()
        B = (sp.eye(tot_prod, 1, k=2)).tocsr() #k-diag is ones, but outside, so this is zero matrix
        C = (sp.eye(1,tot_prod,k=-1)).tocsr()
        x0 = p_occ*numpy.ones((tot_prod,1))
        P0 = (big_covar*sp.eye(tot_prod,tot_prod)).tocsr()
        Q = (small_covar*sp.eye(tot_prod,tot_prod)).tocsr()
        R = (sp.eye(tot_prod, tot_prod)).tocsr()
        self.kf = KalmanFilter(A=A, B=B, C=C, x0=x0, P0=P0, Q=Q, R=R)

    def __call__(self, sonar_measurement):
        C = self.calc_C(sonar_measurement.indices)
        # Convert probabilities to log odds format first
        prob = logodds(sonar_measurement.prob)
        prob = prob[:, numpy.newaxis]
        n = sonar_measurement.var.shape[0]
        R = sp.spdiags(sonar_measurement.var, 0, n, n).tocsr()
        return self.kf.meas_update(prob, C, R)
    
    def time_update(self):
        self.kf.time_update()
    
    def calc_C(self, indices):
        nbr_y = numpy.size(indices,0)
#        C = numpy.zeros((nbr_y, numpy.prod(self.shape)))
#        for i in range(nbr_y):
#            # Convert to linear indexing
#            ind = indices[i, 1]*self.shape[0]+indices[i, 0]
#            C_old[i,ind] = 1
        y = range(nbr_y)
        x = indices[:, 2]*self.shape[1]*self.shape[0] + indices[:, 1]*self.shape[0]+indices[:, 0]
        C_data = ((1,)*nbr_y, (y, x))
        C_shape = (nbr_y, numpy.prod(self.shape))
        C = sp.coo_matrix(C_data, shape=C_shape)
        return C.tocsr()

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

    def get_variance(self, axis=None):
        tmp = self.kf.P.diagonal()
        tmp = numpy.reshape(tmp, self.shape, order='F')
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
