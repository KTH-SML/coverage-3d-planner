import numpy as np
import matplotlib.pyplot as plt
import arrow3d as a3d

import utilities as uts
import footprints as fps



class Sensor(object):


    def __init__(
	        self,
	        pos=np.array([0.0,0.0,0.0]),
	        ori=np.array([1.0,0.0,0.0]),
	        fp=fps.ConvexFootprint(),
	        color = 'black',
	        landmarks = []
	        ):
        self._pos = np.array(pos)
        self._ori = uts.normalize(ori)
        self._fp = fp
        self._color = color


    def __str__(self):
	    string = "Sensor object for the coverage planner."
	    string += "\nPosition: " + str(self._pos)
	    string += "\nOrientation: " + str(self._ori)
	    string += "\nFootprint: " + str(self._fp)
	    return string


    def perception(self, lmk):
	    p = self._pos
	    n = self._ori
	    q = lmk.pos
	    m = lmk.ori
	    return self._fp(p,n,q,m)


    def coverage(self, landmarks):
	    return sum([self.perception(lmk)
		    for lmk in landmarks], 0.0)


    def per_pos_grad(self, lmk):
	    p = self._pos
	    n = self._ori
	    q = lmk.pos
	    m = lmk.ori
	    return self._fp.pos_grad(p,n,q,m)


    def cov_pos_grad(self, landmarks):
	    return sum([self.per_pos_grad(lmk)
		    for lmk in landmarks], np.zeros(3))


    def per_ori_grad(self, lmk):
	    p = self._pos
	    n = self._ori
	    q = lmk.pos
	    m = lmk.ori
	    return self._fp.ori_grad(p,n,q,m)


    def cov_ori_grad(self, landmarks):
	    return sum([self.per_ori_grad(lmk)
		    for lmk in landmarks], np.zeros(3))


    @property
    def pos(self):
	    return np.array(self._pos)

    @pos.setter
    def pos(self, value):
	    self._pos = np.array(value)

    @property
    def ori(self):
	    return np.array(self._ori)

    @ori.setter
    def ori(self, value):
	    self._ori = uts.normalize(value)
	    
    @property
    def color(self):
        return self._color

    @color.setter
    def color(self, value):
        self._color = value
        
        


    def draw(self,
            draw_orientation=True,
            scale=1.0,
            alpha=1.0,
            color=None
            ):

        if color == None:
            color = self._color
        x, y, z = self._pos
        ax = plt.gca()
        point = ax.scatter(x,y,z,
            color=color,
            alpha=alpha)
        arrow = None
        if draw_orientation:
            vec = scale*self.ori
            arr = a3d.Arrow3D(
                [x, x+vec[0]],
                [y, y+vec[1]],
                [z, z+vec[2]],
                mutation_scale=20,
                lw=1,
                arrowstyle="-|>",
                color=color,
                alpha=alpha
                )
            arrow = ax.add_artist(arr)
        return point, arrow
        
        
        






if __name__ == '__main__':
	import landmark as lm
	lmk = lm.Landmark(pos=np.array([1,0,0]))
	srr = Sensor()
	print srr.perception(lmk)
	print srr.coverage([lmk])
	print srr.per_pos_grad(lmk)
	print srr.cov_pos_grad([lmk])
	print srr.per_ori_grad(lmk)
	print srr.cov_ori_grad([lmk])
	plt.figure()
	ax = plt.gca(projection='3d')
	ax.set_xlim3d(-3,3)
	ax.set_ylim3d(-3,3)
	ax.set_zlim3d(-3,3)
	lmk.draw()
	srr.draw()
	plt.grid()
	plt.show()