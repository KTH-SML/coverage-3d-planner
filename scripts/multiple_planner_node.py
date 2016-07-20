#! /usr/bin/env python


import rospy as rp
import coverage_3d_planner.msg as cms
import coverage_3d_planner.srv as csv
import std_msgs.msg as sms

import numpy as np
import threading as thd
import random as rdm

import landmark as lm
import sensor as sn
import footprints as fp
import utilities as uts



lock = thd.Lock()
landmarks = set()
sensor = sn.Sensor(fp=fp.ConvexFootprint())



rp.init_node('planner_node')

KP = rp.get_param('position_gain', 3.0)
KN = rp.get_param('orientation_gain', 1.0)
SP = rp.get_param('velocity_saturation', 1.0)
SN = rp.get_param('angular_velocity_saturation', 0.3)

NAMES = rp.get_param('/names').split()
MY_NAME = rp.get_param('name')
PARTNERS = filter(lambda x: not x == MY_NAME, NAMES)
possible_partners = list(PARTNERS)

XLIM = rp.get_param('xlim', (-5,5))
YLIM = rp.get_param('ylim', (-5,5))
ZLIM = rp.get_param('zlim', (-5,5))

vel_pub = rp.Publisher('cmd_vel', cms.Velocity, queue_size=10)
lmks_pub = rp.Publisher('landmarks', cms.LandmarkArray, queue_size=10)
cov_pub = rp.Publisher('coverage', sms.Float64, queue_size=10)

rp.wait_for_service('/draw_landmarks')
draw_landmarks_proxy = rp.ServiceProxy(
    '/draw_landmarks',
    csv.DrawLandmarks)
    



#def yield_landmarks_handler(req):
#    global landmarks
#    global sensor
#    global lock
#    global PARTNERS, possible_partners
#    your_pos = np.array(req.position)
#    your_ori = np.array(req.orientation)
#    you = sn.Sensor(pos=your_pos, ori=your_ori, fp=fp.ConvexFootprint())
#    your_lmks = [lm.Landmark.from_msg(lmkmsg) for lmkmsg in req.landmarks]
#    mine = set()
#    yours = set()
#    lock.acquire()
#    for lmk in your_lmks:
#        if sensor.perception(lmk) < you.perception(lmk):
#            mine.add(lmk)
#        else:
#            yours.add(lmk)
#    if len(mine)>0:
#        possible_partners = list(PARTNERS)
#        landmarks |= mine
#    lock.release()
#    return csv.TradeLandmarksResponse(
#        success = len(mine)>0,
#        your_landmarks=[lmk.to_msg() for lmk in yours])
#    
#yield_srv = rp.Service(
#    'yield_landmarks',
#    csv.TradeLandmarks,
#    yield_landmarks_handler)

#yield_proxies = dict()
#for partner in PARTNERS:
#    rp.wait_for_service('/'+partner+'/yield_landmarks')
#    yield_proxies[partner] = rp.ServiceProxy(
#        '/'+partner+'/yield_landmarks', csv.TradeLandmarks)



def trade_landmarks_handler(req):
    global landmarks
    global sensor
    global lock
    global PARTNERS, possible_partners
    global MY_NAME
    your_pos = np.array(req.pose.position)
    your_ori = np.array(req.pose.orientation)
    you = sn.Sensor(pos=your_pos, ori=your_ori, fp=fp.ConvexFootprint())
    your_lmks = [lm.Landmark.from_msg(lmkmsg) for lmkmsg in req.landmarks]
    mine = set()
    yours = set()
    success = False
    lock.acquire()
    for lmk in landmarks:
        if you.perception(lmk) < sensor.perception(lmk):
            yours.add(lmk)
            success = True
        else:
            mine.add(lmk)
    for lmk in your_lmks:
        if sensor.perception(lmk) < you.perception(lmk):
            mine.add(lmk)
            success = True
        else:
            yours.add(lmk)
    if success:
        landmarks = mine
        possible_partners = list(PARTNERS)
    lock.release()
    msg = csv.DrawLandmarksRequest(
        name = MY_NAME,
        landmarks = [lmk.to_msg() for lmk in landmarks])
    draw_landmarks_proxy(msg)
    return csv.TradeLandmarksResponse(
        success = success,
        your_landmarks=[lmk.to_msg() for lmk in yours])
    
trade_srv = rp.Service(
    'trade_landmarks',
    csv.TradeLandmarks,
    trade_landmarks_handler)

trade_proxies = dict()
for partner in PARTNERS:
    rp.wait_for_service('/'+partner+'/trade_landmarks')
    trade_proxies[partner] = rp.ServiceProxy(
        '/'+partner+'/trade_landmarks', csv.TradeLandmarks)






# def add_landmark_arc_handler(req):
#     global landmarks
#     global lock
#     global PARTNERS, possible_partners
#     num = req.number
#     rd = req.radius
#     th1 = req.thetamin
#     th2 = req.thetamax
#     lmks = set()
#     for th in np.linspace(np.pi*th1,np.pi*th2,num):
#         ori = np.array([np.cos(th), np.sin(th)])
#         pos = rd*ori
#         lmk = lm.Landmark(pos=pos, ori=ori)
#         lmks.add(lmk)
#     lock.acquire()
#     if len(lmks)>0:
#         possible_partners = list(PARTNERS)
#         landmarks |= lmks
#     lock.release()
#     msg = csv.DrawLandmarksRequest(
#         name = MY_NAME,
#         landmarks = [lmk.to_msg() for lmk in landmarks])
#     draw_landmarks_proxy(msg)
#     return csv.AddLandmarkArcResponse()

# add_lma_srv = rp.Service(
#     'add_landmark_arc',
#     csv.AddLandmarkArc,
#     add_landmark_arc_handler)





def add_random_landmarks_handler(req):
    global landmarks
    global lock
    global PARTNERS, possible_partners
    new_lmks = set()
    for index in range(req.num):
        new_lmks.add(lm.Landmark.random(
	        xlim=0.5*np.array(XLIM),
	        ylim=0.5*np.array(YLIM),
            zlim=0.5*np.array(ZLIM)))
    lock.acquire()
    if req.num>0:
        possible_partners = list(PARTNERS)
        rp.logwarn(MY_NAME + ': reset partners: ' + str(possible_partners))
    landmarks |= new_lmks
    lock.release()
    msg = csv.DrawLandmarksRequest(
        name = MY_NAME,
        landmarks = [lmk.to_msg() for lmk in landmarks])
    draw_landmarks_proxy(msg)
    return csv.AddRandomLandmarksResponse()
    
add_lmk_srv = rp.Service(
    'add_random_landmarks',
    csv.AddRandomLandmarks,
    add_random_landmarks_handler
)



def change_gains_handler(req):
    global KP, KN
    global lock
    lock.acquire()
    KP = req.position_gain
    KN = req.orientation_gain
    lock.release()
    return csv.ChangeGainsResponse()
chg_gns_srv = rp.Service(
    'change_gains',
    csv.ChangeGains,
    change_gains_handler)


def pose_cb(pose):
    global sensor
    global lock
    p = np.array(pose.position)
    n = np.array(pose.orientation)
    lock.acquire()
    sensor.pos = p
    sensor.ori = n
    lock.release()
pose_sub = rp.Subscriber(
    'pose',
    cms.Pose,
    pose_cb)


def add_landmark_handler(req):
    global landmarks
    global lock
    global PARTNERS, possible_partners
    lmk = lm.Landmark.from_msg(req.landmark)
    lock.acquire()
    possible_partners = list(PARTNERS)
    landmarks.add(lmk)
    lock.release()
    msg = csv.DrawLandmarksRequest(
        name = MY_NAME,
        landmarks = [lmk.to_msg() for lmk in landmarks])
    draw_landmarks_proxy(msg)
    return csv.AddLandmarkResponse()
    
add_lmk_srv = rp.Service(
    'add_landmark',
    csv.AddLandmark,
    add_landmark_handler)


def change_landmarks_handler(req):
    global landmarks
    global lock
    lmks = [lm.Landmark.from_message(lmk_msg) for lmk_msg in req.landmarks]
    lock.acquire()
    landmarks = set(lmks)
    lock.release()
    msg = csv.DrawLandmarksRequest(
        name = MY_NAME,
        landmarks = [lmk.to_msg() for lmk in landmarks])
    draw_landmarks_proxy(msg)
    return csv.ChangeLandmarksResponse()
    
chg_lmk_srv = rp.Service(
    'change_landmarks',
    csv.ChangeLandmarks,
    change_landmarks_handler
)






#def smart_gain(coverage, gmin, gmax):
#    return gmin + 2*(gmax-gmin)/(1+np.exp(0.5*coverage))



FAST_RATE = rp.Rate(6e1)
SLOW_RATE = rp.Rate(0.5)

def work():
    global sensor, landmarks
    global vel_pub, lmks_pub
    global KP, KN
    global lock
    global last_trade_time
    global MY_NAME, PARTNERS, possible_partners
    lock.acquire()
    p = sensor.pos
    n = sensor.ori
    if len(landmarks) == 0:
        v = np.zeros(2)
        w = np.zeros(3)
    else:
        #v = -smart_gain(coverage,KP/10,KP)*sensor.cov_pos_grad(landmarks)
        v = -KP/float(len(landmarks))*sensor.cov_pos_grad(landmarks)
        v = uts.saturate(v,SP)
        #w = -smart_gain(coverage,KN/10,KN)*np.cross(n, sensor.cov_ori_grad(landmarks))
        w = KN/float(len(landmarks))*np.cross(n, sensor.cov_ori_grad(landmarks))
        w = uts.saturate(w, SN)
    coverage = sensor.coverage(landmarks)
    if np.linalg.norm(v)+np.linalg.norm(w) < 0.03:
        v = np.zeros(3)
        w = np.zeros(3)
        rp.logwarn(MY_NAME + ': possible partners: ' + str(possible_partners))
        if len(possible_partners)>0:
            pose = cms.Pose(position=p.tolist(), orientation=n.tolist())
            request = csv.TradeLandmarksRequest(pose, [lmk.to_msg() for lmk in landmarks])
            partner = rdm.choice(possible_partners)
            rp.logwarn(MY_NAME + ': request trade with ' + partner)
            try:
                response = trade_proxies[partner](request)
                if response.success:
                    possible_partners = list(PARTNERS)
                    landmarks = set([
                        lm.Landmark.from_msg(item) for item in response.your_landmarks])
                    msg = csv.DrawLandmarksRequest(
                        name = MY_NAME,
                        landmarks = [lmk.to_msg() for lmk in landmarks])
                    draw_landmarks_proxy(msg)
                else:
                    possible_partners.remove(partner)
            except Exception:
                rp.logwarn(MY_NAME + ': ' + partner + ' is unavailable')
        else:
            rp.logwarn(MY_NAME + ': no possible partners')
        rate = SLOW_RATE
    else:
        rate = FAST_RATE
    #lmks_msg = [lmk.to_msg() for lmk in landmarks]
    lock.release()
    cov_vel = cms.Velocity(linear=v.tolist(), angular=w.tolist())
    vel_pub.publish(cov_vel)
    #lmks_pub.publish(lmks_msg)
    cov_pub.publish(coverage)
    rate.sleep()








if __name__ == '__main__':
    while not rp.is_shutdown():
        work()
        