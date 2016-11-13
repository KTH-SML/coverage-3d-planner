#! /usr/bin/env python


import rospy as rp
import coverage_3d_planner.msg as cms

import utilities as uts

import numpy as np
import scipy.linalg as spl



rp.init_node('simulator_node')
pose_pub = rp.Publisher('pose', cms.Pose, queue_size=10)

lin_vel = np.zeros(3)
ang_vel = np.zeros(3)
position = np.array([float(num) for num in rp.get_param('initial_position').split()])
rp.logwarn(position)
orientation = np.array(rp.get_param('initial_orientation', np.eye(3).tolist()))
#rp.logwarn(orientation)
time = rp.get_time()

def vel_cb(vel):
    global lin_vel, ang_vel
    lin_vel = np.array(vel.linear)
    ang_vel = np.array(vel.angular)

vel_sub = rp.Subscriber('cmd_vel', cms.Velocity, vel_cb)

def work():
    global lin_vel, ang_vel
    global position, orientation
    global time
    new_time = rp.get_time()
    dt = new_time - time
    position += lin_vel*dt
    for i in range(3):
        #rp.logwarn(orientation)
        orientation[:,i] += uts.skew(orientation[:,i]).dot(ang_vel)*dt
    orientation = uts.rotation_fix(orientation)
    det = np.linalg.det(orientation)
    #rp.logwarn(det)
    assert det > 0.0
    pose = cms.Pose(
        p=position.tolist(),
        x=orientation[:,0].tolist(),
        y=orientation[:,1].tolist(),
        z=orientation[:,2].tolist()
        )
    pose_pub.publish(pose)
    time = new_time

rate = rp.Rate(6e1)

while not rp.is_shutdown():
    work()
    rate.sleep()
