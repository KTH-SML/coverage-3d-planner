import numpy as np
import scipy.linalg as spl
import matplotlib.pyplot as plt
import random as rdm
import arrow3d as a3d

import utilities as uts
import coverage_3d_planner.msg as cms






class Landmark(object):


    def __init__(self,
            pos=np.zeros(3),
            ori=np.eye(3),
            color='black'
            ):
        self._pos = np.array(pos)
        self._ori = np.array(ori)
        self._color = color


    @classmethod
    def from_msg(cls, msg):
        pos = np.array(msg.p)
        ori = np.eye(3)
        ori[:,0] = msg.x
        ori[:,1] = msg.y
        ori[:,2] = msg.z
        #ori = uts.rotation_fix(ori)
        return cls(pos, ori)


    @classmethod
    def random(cls, xlim, ylim, zlim):
        x = rdm.uniform(*xlim)
        y = rdm.uniform(*ylim)
        z = rdm.uniform(*zlim)
        pos = np.array([x,y,z])
        ori = np.random.rand(3,3)-0.5*np.ones((3,3))
        ori = uts.rotation_fix(ori)
        return cls(pos, ori)


    def to_msg(self):
        p = self.pos.tolist()
        x, y, z = [self.ori[:,i].tolist() for i in range(3)]
        msg = cms.Landmark(p=p, x=x, y=y, z=z)
        return msg


    def __str__(self):
        string = "Landmark object for the coverage planner"
        string += "\nPosition: " + str(self._pos)
        string += "\nOrientation: " + str(self._ori)
        return string


    @property
    def pos(self):
        return np.array(self._pos)


    @pos.setter
    def pos(self, value):
        self._pos = np.array(value)


    @property
    def ori(self):
        return self._ori


    @ori.setter
    def ori(self, value):
        self._ori = value


    @property
    def color(self):
        return self._color


    @color.setter
    def color(self, value):
        self._color = value


    def draw(self,
            draw_orientation=False,
            scale=1.0,
            alpha=1.0,
            color=None
            ):
        if color == None:
            color = self._color
        x, y, z = self._pos
        ax = plt.gca(projection='3d')
        point = ax.scatter(x,y,z,
            color=color,
            alpha=alpha)
        arrows = list()
        if draw_orientation:
            for j in range(3):
                if j == 0:
                    alph = alpha
                else:
                    alph = alpha*0.2
                vec = scale*self.ori[:,j]
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
    plt.figure()
    ax = plt.gca(projection='3d')
    ax.set_xlim3d(-3.0,3.0)
    ax.set_ylim3d(-3.0,3.0)
    ax.set_zlim3d(-3.0,3.0)
    lmk = Landmark()
    lmk.draw()
    print lmk.pos
    print lmk.ori
    print lmk.to_msg()
    plt.show()
