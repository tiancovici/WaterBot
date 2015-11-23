#!/usr/bin/env python
import roslib

roslib.load_manifest('no_weights')
import rospy as rp

rp.init_node('tests')
import mcl_tools

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# PAR_COUNT = 1500
PAR_COUNT = 750

cmd_vel = None

std_hit = 0.5
MAX_DIST = mcl_tools.LASER_MAX

last_time = None

# the weights to care about each of the readings
Z_MAX = 0.0
Z_HIT = 1.0
Z_SHORT = 0.0
Z_RAND = 0.0


# State
parset = [(-25.0, 4.0, 0.0)]

# particle_filter(particle set, action taken, the laser readings)
def particle_filter(ps, control, scan):
    return ps


def got_scan(msg):
    global parset
    global cmd_vel

    control = (0.5, 0.1)

    parset = particle_filter(parset, control, msg)
    mcl_tools.show_particles(parset)

    cmd = Twist()
    (cmd.linear.x, cmd.angular.z) = control
    cmd_vel.publish(cmd)


if __name__ == '__main__':
    # Uncomment for debugging if nesssary, recomment before turning in.
    # rp.Subscriber('/stage/base_pose_ground_truth', Odometry, mcl_debug.got_odom)

    rp.Subscriber('/robot/base_scan', LaserScan, got_scan)
    cmd_vel = rp.Publisher('/robot/cmd_vel', Twist, queue_size=1)

    mcl_tools.mcl_init('no_weights')
    print 'mcl_tools.map_range((2.0,9.0,0.0), 0.0)', mcl_tools.map_range((2.0, 9.0, 0.0), 0.0)
    print 'mcl_tools.map_hit(2.0,9.0)', mcl_tools.map_hit(2.0, 9.0)

    print 'mcl_tools.map_range((8.0,9.0,0.0), 0.0)', mcl_tools.map_range((8.0, 9.0, 0.0), 0.0)
    print 'mcl_tools.map_hit(8.0,9.0)', mcl_tools.map_hit(8.0, 9.0)

    mcl_tools.mcl_run_viz()
