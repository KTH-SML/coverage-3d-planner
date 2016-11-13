import numpy as np
import random as rdm
import matplotlib.pyplot as plt

import landmark as lm
import sensor as sn

import rospy as rp


NUM_LANDMARKS = 100
RADIUS = 1.0
HEIGHT = 5.0
NUM_TURNS = 2.0

XLIM = (-5.0,5.0)
YLIM = (-5.0,5.0)

INIT_LANDMARKS = set()

NAMES = rp.get_param('/names', default="Axel Bo Calle David").split()
INIT_ASSIGNMENT = dict()
for name in NAMES:
	INIT_ASSIGNMENT[name] = set()


for idx in range(NUM_LANDMARKS):
	th = 2.0*np.pi*float(idx)/float(NUM_LANDMARKS)*float(NUM_TURNS) + np.pi/2
	x = RADIUS*np.cos(th)
	y = RADIUS*np.sin(th)
	z = HEIGHT*float(idx)/float(NUM_LANDMARKS) - HEIGHT/2.0
	pos = np.array([x, y, z])
	ori = np.eye(3)
	ori[0,0] = -np.cos(th)
	ori[1,0] = -np.sin(th)
	ori[0,1] = np.sin(th)
	ori[1,1] = -np.cos(th)
	rp.logwarn(ori)
	#ori[0:2,1] = [-np.sin(th), np.cos(th)]
	lmk = lm.Landmark(pos=pos, ori=ori)
	INIT_LANDMARKS.add(lmk)
	INIT_ASSIGNMENT[NAMES[0]].add(lmk)



if __name__ == '__main__':
	plt.figure()
	for lmk in INIT_LANDMARKS:
		lmk.draw()
	sns = sn.Sensor()
	sns.contour_plot(
		INIT_LANDMARKS,
		xlim=XLIM,
		ylim=YLIM)
	plt.xlim(XLIM)
	plt.ylim(YLIM)
	plt.show()


# NUM_LANDMARKS_SQRT = 25
# XLIM = (-3,3)
# YLIM = (-3,
#
# NAMES = rp.get_param('/names').split()
#
# LANDMARKS = dict()
# for name in NAMES:
# 	LANDMARKS[name] = set()
#
# rdm.seed(89)
#
# for index in range(NUM_LANDMARKS_SQRT**2):
# 	x = float(index%NUM_LANDMARKS_SQRT)*(
# 		XLIM[1]-XLIM[0])/NUM_LANDMARKS_SQRT+XLIM[0]
# 	y = float(index//NUM_LANDMARKS_SQRT)*(
# 		YLIM[1]-YLIM[0])/NUM_LANDMARKS_SQRT+YLIM[0]
# 	pos = (x,y)
# 	ori = (1,0)
# 	lmk = lm.Landmark(pos, ori)
# 	#landmarks[rdm.choice(landmarks.keys())].add(lmk)
# 	LANDMARKS[NAMES[0]].add(lmk)


if __name__ == '__main__':
	print landmarks
