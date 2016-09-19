import numpy as np
import matplotlib.pyplot as plt
import warnings as wrn






class Footprint(object):
    """The Footprint class.
    Abstract class.
    This class implements an abstract footprint object
    for the coverage planner.
    """


    def __str__(self):
        string = self.__class__.__name__
        return string


    def __call__(self, ps, Rs, pl, Rl):
        """Function that returns the value of the footprint.
        
        Args:
            ps: Position of the sensor.
            Rs: Orientation of the sensor.
            pl: Position of the landmark.
            Rl: Orientation of the landmark.
            
        Returns:
            float: value of the footprint (>=0).
        
        """
        raise NotImplementedError()


        
    def pos_grad(self, ps, Rs, pl, Rl):
        """Function that returns the gradient of the footprint
        with respect to the position.
        
        Args:
            ps: Position of the sensor.
            Rs: Orientation of the sensor.
            pl: Position of the landmark.
            Rl: Orientation of the landmark.
            
        Returns:
            numpy array: gradient of the footprint
                with respect to the position.
        
        """
        raise NotImplementedError()

        
    def ori_grad(self, ps, Rs, pl, Rl):
        """Function that returns the gradient of the footprint
        with respect to the orientation.
        
        Args:
            ps: Position of the sensor.
            Rs: Orientation of the sensor.
            pl: Position of the landmark.
            Rl: Orientation of the landmark.
            
        Returns:
            numpy array 3-by-3:
                gradient of the footprint
                with respect to the orientation.
        
        """
        raise NotImplementedError()


        
    def contour_plot(
            self,
            ps=np.zeros(3),
            Rs=np.eye(3),
            plz_func = None,
            Rl_func = None,
            xlim = None,
            ylim = None,
            num_points = 100,
            filename = None
            ):
        """Draw a contour plot of the footprint
        as a function of xy position of the landmark.
        
        Args:
            ps: Position of the sensor.
            Rs: Orientation of the sensor.
            qz_func:
                Function that returns qz
                as a function of plx and ply (default returns ps[2])
            Rl_func:
                Function that returns the orientation of the landmark
                as a function of pl (default returns Rs)
            xlim: plot limits around ps[0]
            ylim: plot limits around ps[1]
            num_points: number of points generated for the plot (default 100)
            filename: name of the file where the plot is saved
                (default '[ClassName].pdf')
        
        """
        if Rl_func == None:
            def Rl_func(plx, ply):
                return Rs
        if plz_func == None:
            def plz_func(plx, ply):
                return ps[2]
        if filename == None:
            filename = self.__class__.__name__ + '.pdf'
        if xlim == None:
            xlim = (-1.0,1.0)
        if ylim == None:
            ylim = tuple(xlim)
        xvec = np.linspace(ps[0]+xlim[0], ps[0]+xlim[1], num=num_points)
        yvec = np.linspace(ps[1]+ylim[0], ps[1]+ylim[1], num=num_points)
        zmat = np.zeros((len(xvec), len(yvec)))
        for i, x in enumerate(xvec):
	        for j, y in enumerate(yvec):
		        pl = np.array([x, y, plz_func(x,y)])
		        Rl = Rl_func(x, y)
		        zmat[j][i] = self(ps, Rs, pl, Rl)
        cs = plt.contour(xvec,yvec,zmat,20)
        plt.xlabel('$p_x$')
        plt.ylabel('$p_y$')
        #plt.axis('equal')
        plt.colorbar(cs)
        plt.grid()
        plt.savefig(filename)
        







class SphericalFootprint(Footprint):

    def __init__(self, best_distance=1.0, ori_weight=1.0):
        self._BEST_DISTANCE = best_distance
        self._ORI_WEIGHT = ori_weight
        
    def __call__(self, ps, Rs, pl, Rl):
        BD = self._BEST_DISTANCE
        OW = self._ORI_WEIGHT
        result = 0.0
        result += np.linalg.norm(ps+BD*Rs[:,0]-pl)**2
        result += OW*np.linalg.norm(ps+BD*Rl[:,0]-pl)**2
        return result
        
    def pos_grad(self, ps, Rs, pl, Rl):
        BD = self._BEST_DISTANCE
        OW = self._ORI_WEIGHT
        result = np.zeros(3,1)
        result += 2*ps+2*BD*Rs[:,0]-2*pl
        result += OW*(2*ps+2*BD*Rl[:,0]-2*pl)
        return result
        
    def ori_grad(self, ps, Rs, pl, Rl):
        BD = self._BEST_DISTANCE
        OW = self._ORI_WEIGHT
        result = np.zeros(3,3)
        result[:,0] += (2*ps+2*BD*Rs[:,0]-2*pl)*BD
        result[:,0] += (2*ps+2*BD*Rl[:,0]-2*pl)*BD
        return result
        
        
    def contour_plot(self):
        BD = self._BEST_DISTANCE
        xlim = (-0.5*BD, 1.5*BD)
        ylim = (-BD, BD)
        Footprint.contour_plot(self, xlim=xlim, ylim=ylim)
        
        
        
        



class EggFootprint(Footprint):

    def __init__(self,
            best_distance=1.0,
            front_gain=0.01,
            rear_gain=1.0,
            facet_gain=0.0):
        if rear_gain < front_gain:
            wrn.warn('The rear gain is smaller than the front gain.')
        self._BEST_DISTANCE = best_distance
        self._FRONT_GAIN = front_gain
        self._REAR_GAIN = rear_gain
        self._FACET_GAIN = facet_gain
        
    def __call__(self, ps, Rs, pl, Rl):
        BD = self._BEST_DISTANCE
        FG = self._FRONT_GAIN
        RG = self._REAR_GAIN
        MG = self._FACET_GAIN
        vec = ps+BD*Rs[:,0]-pl
        norm = np.linalg.norm(vec)
        one = (FG+RG)/2*norm**2
        two = (RG-FG)/2*norm*vec.dot(Rs[:,0])
        vec = ps+BD*Rl[:,0]-pl
        norm = np.linalg.norm(vec)
        three = (FG+RG)/2*norm**2
        four = (RG-FG)/2*norm*vec.dot(Rl[:,0])
        return one + two + MG*(three + four)
        
    def pos_grad(self, ps, Rs, pl, Rl):
        BD = self._BEST_DISTANCE
        FG = self._FRONT_GAIN
        RG = self._REAR_GAIN
        MG = self._FACET_GAIN
        vec = ps+BD*Rs[:,0]-pl
        norm = np.linalg.norm(vec)
        one = (FG+RG)*vec
        if norm==0:
            two = np.zeros(3)
        else:
            two = (RG-FG)/2*(vec.dot(Rs[:,0])*vec/norm+norm*Rs[:,0])
        vec = ps+BD*Rl[:,0]-pl
        norm = np.linalg.norm(vec)
        three = (FG+RG)*vec
        if norm==0:
            four = np.zeros(3)
        else:
            four = (RG-FG)/2*(vec.dot(Rl[:,0])*vec/norm+norm*Rl[:,0])
        return one + two + MG*(three + four)
        
    def ori_grad(self, ps, Rs, pl, Rl):
        BD = self._BEST_DISTANCE
        FG = self._FRONT_GAIN
        RG = self._REAR_GAIN
        vec = ps+BD*Rs[:,0]-pl
        norm = np.linalg.norm(vec)
        one = (FG+RG)*vec
        if norm==0:
            two = np.zeros(3)
        else:
            two = (RG-FG)/2*(vec.dot(Rs[:,0])*vec/norm+norm*Rs[:,0]+norm*vec)
        x = BD*(one + two)
        y = np.zeros(3)
        z = np.zeros(3)
        return x, y, z
        
    def contour_plot(self, **kwargs):
        BD = self._BEST_DISTANCE
        xlim = (-BD+BD,4.0*BD+BD)
        ylim = (-BD,BD)
        Footprint.contour_plot(self, xlim=xlim, ylim=ylim, **kwargs)
        
        


"""Test"""
if __name__ == '__main__':
    fp = EggFootprint()
    print fp
    plt.figure()
    #def Rl[:,0]_func(pl):
    #    return np.array([0,1,0])
    #fp.contour_plot(Rl[:,0]_func=Rl[:,0]_func)
    fp.contour_plot()
    plt.show()
