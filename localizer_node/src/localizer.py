#!/usr/bin/env python
import numpy as np

import rospy as rp

rp.init_node('localizer_node')
import mcl_tools
from math import atan2
from functools import partial

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

PAR_COUNT = 1500
cmd_vel = None

STD_DEV_HIT = 1.0
MAX_DIST = mcl_tools.LASER_MAX
Z_HIT = 0.6
Z_RAND = 0.3
Z_MAX = 0.1

last_time = None
angles = None

# State
parset = [mcl_tools.random_particle() for ii in range(PAR_COUNT)]


# get the weight of a single particle given a laser scan
def particle_weight(scan, particle):
    scan_min = scan.angle_min
    scan_inc = scan.angle_increment

    prob = 1.0
    for i in xrange(len(scan.ranges)):
        sensed = scan.ranges[i]
        traced = mcl_tools.map_range(particle, scan_min + (i * scan_inc))
        prob *= Z_HIT * p_hit(sensed, traced) \
                + Z_RAND * p_rand(sensed, traced) \
                + Z_MAX * p_max(sensed, traced)

    return prob


# particle_filter(particle set, action taken, the laser readings)
def particle_filter(ps, control, scan):
    global last_time, PAR_COUNT
    if last_time is None:
        last_time = rp.get_rostime()

    # probabilistically move all the particles
    rp.loginfo((rp.get_rostime() - last_time).to_sec())
    new_pos = partial(integrate_control_to_distance, control, (rp.get_rostime() - last_time).to_sec())
    last_time = rp.get_rostime()
    ps = [new_pos(part) for part in ps if not mcl_tools.map_hit(part[0], part[1])]

    # replace pixels that are outside of the map with new randomly placed pixels
    while len(ps) < PAR_COUNT:
        part = mcl_tools.random_particle()
        if not mcl_tools.map_hit(part[0], part[1]):
            ps.append(part)

    weights = map(partial(particle_weight, scan), ps)  # get the weights for each particle
    weights = np.multiply(weights, 1.0 / np.sum(weights))  # normalize the weights

    return mcl_tools.random_sample(ps, PAR_COUNT, weights)


# convert polar control and initial position into a probabilistically determined cartesian position
def integrate_control_to_distance(polar_control, dt, original_pos):
    (xo, yo, to) = original_pos
    (vl, vt) = polar_control

    std_dev = np.sqrt(vl ** 2 + vt ** 2)
    if std_dev == 0.0:
        std_dev = 1.0

    vl_gauss = np.random.normal(vl, std_dev)  # normalized linear velocity
    vt_gauss = np.random.normal(vt, std_dev)  # normalized angular velocity

    t = to + (vt_gauss * dt)  # new angle
    t_mid = (t + to) / 2.0  # take the average of the final and initial angle to for the cartesian translation

    dx = vl_gauss * np.cos(t_mid) * dt
    dy = vl_gauss * np.sin(t_mid) * dt

    # print 'vl', vl, 'vt', vt, 'dy', dy, 'dx', dx, 'stddev', std_dev, 'dt', dt
    x = xo + dx
    y = yo + dy

    return x, y, t

# precomputes the coefficient and exponent base for faster normal calculations
p_hit_coeff = 1. / (np.sqrt(2 * np.pi * STD_DEV_HIT * STD_DEV_HIT))
p_hit_exp = np.exp(-1. / (2. * STD_DEV_HIT * STD_DEV_HIT))


def p_hit(sensed_distance, raytraced_distance):
    global p_hit_coeff, p_hit_exp
    if sensed_distance > MAX_DIST:
        return 0.0
    return p_hit_coeff * (p_hit_exp ** ((sensed_distance - raytraced_distance) ** 2))


def p_max(sensed_distance, raytraced_distance):
    global MAX_DIST
    if sensed_distance == MAX_DIST:
        return 1.0
    return 0.0


def p_rand(sensed_distance, raytraced_distance):
    if sensed_distance < MAX_DIST:
        return 0.0
    return 1. / MAX_DIST


def got_scan(msg):
    global parset, cmd_vel, angles

    if angles is None:  # statically use it after initialization (message shouldn't change length while it's running)
        angles = np.multiply(range(len(msg.ranges)), msg.angle_increment) + msg.angle_min

    measurements = zip(msg.ranges, angles)
    fx = np.sum([l * np.cos(t) for l, t in measurements])
    fy = np.sum([l * np.sin(t) for l, t in measurements])

    # linear_speed = 0.6*(msg.ranges[2]) # scale the speed by the range on the front sensor
    linear_speed = 0.2 * (msg.ranges[2])  # scale the speed by the range on the front sensor
    angular_speed = (10. / msg.ranges[2]) * atan2(fy, fx)
    control = (linear_speed, angular_speed)
    rp.loginfo('control ' + str(control))

    parset = particle_filter(parset, control, msg)
    mcl_tools.show_particles(parset)

    cmd = Twist()
    (cmd.linear.x, cmd.angular.z) = control
    cmd_vel.publish(cmd)


if __name__ == '__main__':
    # Uncomment for debugging if nesssary, recomment before turning in.
    # rp.Subscriber('/stage/base_pose_ground_truth', Odometry, mcl_debug.got_odom)

    rp.Subscriber('/robot/base_scan', LaserScan, got_scan, queue_size=1)
    # rp.Subscriber('/scan', LaserScan, got_scan, queue_size=1)
    # cmd_vel = rp.Publisher('/robot/cmd_vel', Twist, queue_size=1)

    mcl_tools.mcl_init('localizer_node')
    mcl_tools.mcl_run_viz()
