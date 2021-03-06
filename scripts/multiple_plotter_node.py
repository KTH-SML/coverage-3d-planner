#! /usr/bin/env python


import rospy as rp
import coverage_3d_planner.msg as cms
import coverage_3d_planner.srv as csv

import numpy as np
import matplotlib.pyplot as plt
import threading as thd
import mpl_toolkits.mplot3d as m3d

import landmark as lm
import sensor as sn



rp.init_node('plotter_node')
XLIM = rp.get_param('xlim', (-5,5))
YLIM = rp.get_param('ylim', (-5,5))
ZLIM = rp.get_param('zlim', (-5,5))
NAMES = rp.get_param('names').split()
COLORS = rp.get_param('colors', '#1f618d #cb4335 #b7950b #186a3b').split()
COLDIC = dict()
for index, name in enumerate(NAMES):
    COLDIC[name] = COLORS[index]


plt.ion()
fig = plt.figure(figsize=(15,15))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim3d(*XLIM)
ax.set_ylim3d(*YLIM)
ax.set_zlim3d(*ZLIM)
ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')
ax.view_init(45, 45)



sensor_locks = dict()
landmarks_locks = dict()
for name in NAMES:
    sensor_locks[name] = thd.Lock()
    landmarks_locks[name] = thd.Lock()


sensors = dict()
draw_sensor_flags = dict()
for name in NAMES:
    sensors[name] = sn.Sensor(color=COLDIC[name])
    draw_sensor_flags[name] = True




def pose_cb(pose, name):
    global sensors
    global draw_sensor_flags
    global sensor_locks
    p = np.array(pose.p)
    R = np.eye(3)
    R[:,0] = pose.x
    R[:,1] = pose.y
    R[:,2] = pose.z
    sensor_locks[name].acquire()
    sensors[name].pos = p
    sensors[name].ori = R
    draw_sensor_flags[name] = True
    sensor_locks[name].release()

pose_subs = [
    rp.Subscriber(
        name+'/pose',
        cms.Pose,
        pose_cb,
        callback_args=name)
    for name in NAMES]


landmarks = dict()
draw_landmarks_flags = dict()
for name in NAMES:
    landmarks[name] = set()
    draw_landmarks_flags[name] = True

#def landmarks_cb(msg, name):
#    global landmarks
#    global draw_landmarks_flags
#    global landmarks_locks
#    landmarks_locks[name].acquire()
#    landmarks[name] = [lm.Landmark.from_msg(datum) for datum in msg.data]
#    draw_landmarks_flags[name] = True
#    landmarks_locks[name].release()

#landmarks_subs = [
#    rp.Subscriber(
#        name+'/landmarks',
#        cms.LandmarkArray,
#        landmarks_cb,
#        callback_args=name)
#    for name in NAMES
#]



def draw_landmarks_handler(msg):
    global landmarks
    global draw_landmarks_flags
    global landmarks_locks
    name = msg.name
    landmarks_locks[name].acquire()
    landmarks[name] = [lm.Landmark.from_msg(datum) for datum in msg.landmarks]
    draw_landmarks_flags[name] = True
    landmarks_locks[name].release()
    return csv.DrawLandmarksResponse()

draw_landmarks_service = rp.Service(
    'draw_landmarks',
    csv.DrawLandmarks,
    draw_landmarks_handler)






points = dict()
arrows = dict()
lmks_artists = dict()
for name in NAMES:
    sensor_locks[name].acquire()
    points[name], arrows[name] = sensors[name].draw()
    sensor_locks[name].release()
    landmarks_locks[name].acquire()
    lmks_artists[name] = [
        lmk.draw(
            color=COLDIC[name],
            draw_orientation=False,
            scale=1.0)
        for lmk in landmarks[name]]
    landmarks_locks[name].release()


def work():
    global points, arrows, lmks_artists
    global draw_sensor_flags, draw_landmarks_flags
    global sensors, landmarks
    global sensor_locks, landmarks_locks
    for name in NAMES:
        sensor_locks[name].acquire()
        if draw_sensor_flags[name]:
            points[name].remove()
            if not len(arrows[name]) == 0:
                for arrow in arrows[name]:
                    arrow.remove()
            points[name], arrows[name] = sensors[name].draw()
            draw_sensor_flags[name] = False
        sensor_locks[name].release()
        landmarks_locks[name].acquire()
        if draw_landmarks_flags[name]:
            for lp, las in lmks_artists[name]:
                lp.remove()
                if not len(las) == 0:
                    for la in las:
                        la.remove()
            lmks_artists[name] = [
                lmk.draw(
                    color=COLDIC[name])
                for lmk in landmarks[name]]
            draw_landmarks_flags[name] = False
        landmarks_locks[name].release()
    plt.draw()




rate = rp.Rate(1e1)
if __name__ == '__main__':
    while not rp.is_shutdown():
        work()
        rate.sleep()
