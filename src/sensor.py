import numpy as np
import scipy.linalg as spl
import matplotlib.pyplot as plt
import arrow3d as a3d

import utilities as uts
import footprints as fps



class Sensor(object):


    def __init__(
	        self,
	        pos=np.zeros(3),
	        ori=np.eye(3),
	        fp=fps.EggFootprint(),
            #pos,
            #ori,
            #fp,
	        color = 'black'
	        ):
        self._pos = np.array(pos)
        self._ori = np.array(ori)
        self._fp = fp
        self._color = color


    def __str__(self):
	    string = "Sensor object for the coverage planner."
	    string += "\nPosition: " + str(self._pos)
	    string += "\nOrientation: " + str(self._ori)
	    string += "\nFootprint: " + str(self._fp)
	    return string


    def perception(self, lmk, pos=None, ori=None):
        if pos == None:
            pos = self._pos
        if ori == None:
            ori = self._ori
        q = lmk.pos
        S = lmk.ori
        return self._fp(pos,ori,q,S)


    def coverage(self, landmarks, pos=None, ori=None):
        if pos == None:
            pos = self._pos
        if ori == None:
            ori = self._ori
        return sum([self.perception(lmk, pos, ori)
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
        return sum([self.per_ori_grad(lmk)
		    for lmk in landmarks], np.zeros((3,3)))


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
        self._ori = np.array(value)

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



    def contour_plot(
            self,
            landmarks = set(),
            z_func = None,
            Rs_func = None,
            xlim = None,
            ylim = None,
            num_points = 100,
            filename = None
            ):
        if Rs_func == None:
            def Rs_func(plx, ply):
                return self._ori
        if z_func == None:
            def z_func(plx, ply):
                return self._pos[2]
        if filename == None:
            filename = self.__class__.__name__ + '.pdf'
        if xlim == None:
            xlim = (-1.0,1.0)
        if ylim == None:
            ylim = tuple(xlim)
        xvec = np.linspace(
            self._pos[0]+xlim[0],
            self._pos[0]+xlim[1],
            num=num_points)
        yvec = np.linspace(
            self._pos[1]+ylim[0],
            self._pos[1]+ylim[1],
            num=num_points)
        zmat = np.zeros((len(xvec), len(yvec)))
        for i, x in enumerate(xvec):
	        for j, y in enumerate(yvec):
		        ps = np.array([x, y, z_func(x,y)])
		        Rs = Rs_func(x, y)
		        zmat[j][i] = self.coverage(
                    landmarks=landmarks,
                    pos=ps,
                    ori=Rs)
        print(zmat)
        cs = plt.contour(xvec,yvec,zmat,20)
        plt.xlabel('$p_x$')
        plt.ylabel('$p_y$')
        #plt.axis('equal')
        plt.colorbar(cs)
        plt.grid()
        plt.savefig(filename)






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
