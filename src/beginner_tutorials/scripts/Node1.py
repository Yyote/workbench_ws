from re import L
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rospy.init_node("laser_mover")
pub_vel = rospy.Publisher("/cmd_vel", Twist , queue_size=1)
global l 
l = 1

def lasercb (msg):
    global l 
    l = msg.ranges [179]

def mover(vel):
    vel_ = Twist()
    vel_.linear.x = vel
    pub_vel.publish(vel_)

def controller ():
    global l 
    if l < 0.5:
        mover (0)
    else:
        mover (0.1)

while not rospy.is_shutdown():
    controller()
    rospy.sleep(0.2)

rospy.Subscriber("/scan", LaserScan, lasercb)