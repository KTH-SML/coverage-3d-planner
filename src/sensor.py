import numpy as np
import scipy.linalg as spl
import matplotlib.pyplot as plt
import arrow3d as a3d

import utilities as uts
import footprints as fps



class Sensor(object):


    def __init__(
	        self,
	        pos=np.array([0.0,0.0,0.0]),
	        ori=np.eye(3),
	        fp=fps.EggFootprint(),
            #pos,
            #ori,
            #fp,
	        color = 'black',
	        landmarks = []
	        ):
        self._pos = np.array(pos)
        self._ori = spl.polar(ori)[0]
        self._ori /= spl.det(self._ori)
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
	    R = self._ori
	    q = lmk.pos
	    S = lmk.ori
	    return self._fp(p,R,q,S)


    def coverage(self, landmarks):
	    return sum([self.perception(lmk)
		    for lmk in landmarks], 0.0)


    def per_pos_grad(self, lmk):
	    p = self._pos
	    R = self._ori
	    q = lmk.pos
	    S = lmk.ori
	    return self._fp.pos_grad(p,R,q,S)


    def cov_pos_grad(self, landmarks):
	    return sum([self.per_pos_grad(lmk)
		    for lmk in landmarks], np.zeros(3))


    def per_ori_grad(self, lmk):
	    p = self._pos
	    R = self._ori
	    q = lmk.pos
	    S = lmk.ori
	    return self._fp.ori_grad(p,R,q,S)


    def cov_ori_grad(self, landmarks):
        x = np.zeros(3)
        y = np.zeros(3)
        z = np.zeros(3)
        for lmk in landmarks:
            dx, dy, dz = self.per_ori_grad(lmk)
            x += dx
            y += dy
            z += dz
	    return x, y, z


    @property
    def pos(self):
	    return np.array(self._pos)

    @pos.setter
    def pos(self, value):
	    self._pos = np.array(value)

    @property
    def ori(self):
	    return spl.polar(self._ori)[0]

    @ori.setter
    def ori(self, value):
	    self._ori = spl.polar(value)[0]

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
        arrows = list()
        if draw_orientation:
            for j in range(3):
                vec = scale*self.ori[:,j]
                if j > 0:
                    alph = 0.2*alpha
                else:
                    alph = alpha
                arr = a3d.Arrow3D(
                    [x, x+vec[0]],
                    [y, y+vec[1]],
                    [z, z+vec[2]],
                    mutation_scale=20,
                    lw=1,
                    arrowstyle="-|>",
                    color=color,
                    alpha=alph
                    )
                arrows.append(ax.add_artist(arr))
        return point, arrows









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
