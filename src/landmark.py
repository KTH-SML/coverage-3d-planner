import numpy as np
import matplotlib.pyplot as plt
import random as rdm
import arrow3d as a3d

import utilities as uts
import coverage_3d_planner.msg as cms




class Landmark(object):


    def __init__(self,
            pos=np.array([0.0, 0.0, 0.0]),
            ori=np.array([1.0, 0.0, 0.0]),
            color='black'
            ):
        self._pos = np.array(pos)
        self._ori = uts.normalize(ori)
        self._color = color


    @classmethod
    def from_msg(cls, msg):
        pos = np.array(msg.position)
        ori = uts.normalize(np.array(msg.orientation))
        return cls(pos, ori)

        
    @classmethod
    def random(cls,
            xlim=(-3.0, 3.0),
            ylim=(-3.0, 3.0),
            zlim=(-3.0, 3.0)
            ):
        x = rdm.uniform(*xlim)
        y = rdm.uniform(*ylim)
        z = rdm.uniform(*zlim)
        pos = np.array([x,y,z])
        ori = uts.normalize(np.random.rand(3)-0.5)
        return cls(pos, ori)


    def to_msg(self):
        msg = cms.Landmark(
            position=self._pos.tolist(),
            orientation=self._ori.tolist())
        return msg


    def __str__(self):
        string = "Landmark object for the coverage planner"
        string += "\nPosition: " + str(self._pos)
        string += "\nOrientation: " + str(self._ori)
        return string


    @property
    def pos(self):
        return self._pos


    @pos.setter
    def pos(self, value):
        self._pos = np.array(value)


    @property
    def ori(self):
        return self._ori


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
        ax = plt.gca(projection='3d')
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
    plt.figure()
    ax = plt.gca(projection='3d')
    ax.set_xlim3d(-3.0,3.0)
    ax.set_ylim3d(-3.0,3.0)
    ax.set_zlim3d(-3.0,3.0)
    lmk = Landmark()
    lmk.draw()
    plt.show()
