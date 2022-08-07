#!/usr/bin/env python3

from cmath import inf
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry

[r1,r2,r3,r4,r5]=[0.0,0.0,0.0,0.0,0.0]


def us_msg():
    global r1, r2,r3,r4,r5
def s1(msg):
    us_msg.r1=min(msg.ranges[0:49])
def s2(msg):
    us_msg.r2=min(msg.ranges[0:49])
def s3(msg):
    us_msg.r3=min(msg.ranges[0:49])
def s4(msg):
    us_msg.r4=min(msg.ranges[0:49])
def s5(msg):
    us_msg.r5=min(msg.ranges[0:49])
    callback_laser(us_msg)

def callback_laser(us_msg):
  run = Twist()
  linear_x = 0
  angular_z = 0
  if(us_msg.r1 < 2 ):
        if(us_msg.r2 < 1.5 or us_msg.r3 < 1 ):
            
                linear_x = 0
                angular_z = -0.5
                angular_y = 0
                print("go straight of left case ")
        else:
               linear_x = 0
               angular_z =0.5
               angular_y = 0
               print("go straight of right side ")
          
  else:
            linear_x = 0.6
            angular_z = 0
            angular_y = 0
            print("left side rotate ")
  run.linear.x = linear_x
  run.angular.z = angular_z
  pub1.publish(run)

def main():
  global pub1
  rospy.init_node('reading_laser')

  pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


  sub1 = rospy.Subscriber('/robot/laser/scan_1', LaserScan, s1)
  sub2 = rospy.Subscriber('/robot/laser/scan_2', LaserScan, s2)
  sub3 = rospy.Subscriber('/robot/laser/scan_3', LaserScan, s3)
  sub4 = rospy.Subscriber('/robot/laser/scan_4', LaserScan, s4)
  sub5 = rospy.Subscriber('/robot/laser/scan_5', LaserScan, s5)
 
  rospy.spin()

if __name__ == '__main__':
  main()