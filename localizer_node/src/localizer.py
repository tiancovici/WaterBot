#!/usr/bin/env python
import numpy as np
from functools import partial
import math

import rospy as rp
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


import mcl_tools

rp.init_node('localizer_node')

PAR_COUNT = 200  # total number of particles
MID_UNCERT = int(math.floor(0.20 * PAR_COUNT))  # % of total particles to randomize
MID_LIFE = 30  # How many time steps before adding random particles into array
MID_COUNT = 0
LONG_UNCERT = int(math.floor(0.80 * PAR_COUNT))  # % of total particles to randomize
LONG_LIFE = 80  # How many time steps before adding random particles into array
LONG_COUNT = 0

odometry = None

STD_DEV_HIT = 1.0
MAX_DIST = mcl_tools.LASER_MAX
Z_HIT = 0.6
Z_RAND = 0.3
Z_MAX = 0.1

last_time = None
angles = None

# Initially uncertain
parset = [mcl_tools.random_particle() for ii in range(PAR_COUNT)]


# get the weight of a single particle given a laser scan
def particle_weight(scan, particle):
    scan_min = scan.angle_min
    # scan_inc = scan.angle_increment * 50
    scan_inc = scan.angle_increment

    prob = 1.0
    for i in xrange(len(scan.ranges)):
        if i % 50 is 0:
            # print "scan: " + str(i)
            sensed = scan.ranges[i]
            val = scan_min + (i * scan_inc)
            traced = mcl_tools.map_range(particle, val)
            zhit = Z_HIT * p_hit(sensed, traced)
            zmax = Z_MAX * p_max(sensed, traced)
            zrand = Z_RAND * p_rand(sensed, traced)
            prob *= (zhit + zmax + zrand)

    return prob


# particle_filter(particle set, action taken, the laser readings)
def particle_filter(ps, control, scan):
    global last_time, PAR_COUNT, MID_LIFE, MID_UNCERT, MID_COUNT, \
        LONG_LIFE, LONG_UNCERT, LONG_COUNT

    if last_time is None:
        last_time = rp.get_rostime()

    # probabilistically move all the particles
    new_pos = partial(integrate_control_to_distance, control, (rp.get_rostime() - last_time).to_sec())
    last_time = rp.get_rostime()
    new_ps = []
    for part in ps:
        # part[0] = x
        # part[1] = y
        if not mcl_tools.map_hit(part[0], part[1]):
            # print "hit"
            new_ps.append(new_pos(part))
        else:
            new_ps.append(mcl_tools.random_particle())
    ps = new_ps  # update our particle set

    # update weights of each particle
    weights = []
    for part in ps:
        weight = particle_weight(scan, part)
        weights.append(weight)

    # normalize the weights
    weights = np.multiply(weights, 1.0 / np.sum(weights))

    '''
    ############ RESAMPLING! ##############
    '''
    correction = False

    # mid level corrections
    if MID_COUNT > MID_LIFE:  # crisis
        correction = True
        # resample 20%
        ps = mcl_tools.random_sample(ps, PAR_COUNT - MID_UNCERT, weights)
        rand_ps = []
        for x in range(MID_UNCERT):
            rand_ps.append(mcl_tools.random_particle())
        ps.extend(rand_ps)
        MID_COUNT = 0
        print "mid"
        return ps
    else:
        MID_COUNT += 1

    # big corrections
    if LONG_COUNT > LONG_LIFE:
        correction = True
        # resample 50%
        ps = mcl_tools.random_sample(ps, PAR_COUNT - LONG_UNCERT, weights)
        rand_ps = []
        for x in range(LONG_UNCERT):
            rand_ps.append(mcl_tools.random_particle())
        ps.extend(rand_ps)
        LONG_COUNT = 0
        print "long"
        return ps
    else:
        LONG_COUNT += 1

    # no corrections (normal resample all particles)
    if not correction:
        ps = mcl_tools.random_sample(ps, PAR_COUNT, weights)
        return ps


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
    global parset, odometry

    if odometry is not None:
        # print odometry
        linear_speed = odometry.twist.twist.linear.x
        angular_speed = odometry.twist.twist.angular.z
        control = (linear_speed, angular_speed)
        # print control

        parset = particle_filter(parset, control, msg)  # run particle filter
        mcl_tools.show_particles(parset)  # render particle cloud
    else:
        print "odometry is none"


def got_odom(msg):
    global odometry
    odometry = msg

def got_map(msg):
    print msg

if __name__ == '__main__':
    rp.Subscriber('/scan', LaserScan, got_scan, queue_size=1)
    rp.Subscriber('/odom', Odometry, got_odom, queue_size=1)
    # rp.Subscriber('/map', )

    mcl_tools.mcl_init('localizer_node')
    mcl_tools.mcl_run_viz()
