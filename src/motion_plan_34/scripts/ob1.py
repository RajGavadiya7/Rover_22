#!/usr/bin/env python3
import rospy
from cmath import inf
from tf.transformations import euler_from_quaternion
from math import atan2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry

[r1,r2,r3,r4,r5]=[0.0,0.0,0.0,0.0,0.0]

def main():
    global pub
    rospy.init_node('reading_laser')
    sub1 = rospy.Subscriber('/robot/laser/scan_1', LaserScan, s1)
    sub2 = rospy.Subscriber('/robot/laser/scan_2', LaserScan, s2)
    sub3 = rospy.Subscriber('/robot/laser/scan_3', LaserScan, s3)
    sub4 = rospy.Subscriber('/robot/laser/scan_4', LaserScan, s4)
    sub5 = rospy.Subscriber('/robot/laser/scan_5', LaserScan, s5)
   
    sub = rospy.Subscriber("/odom", Odometry, print_angle)
    
    
    
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    rospy.spin()

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
    

global x ,y,theta
x =0.0
y = 0.0 
theta = 0.0


speed = Twist()

goal = Point()
goal.x = 11
goal.y = -16


def print_angle(msg):
    
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    print("global",theta)

    
    inc_x = goal.x -current_x
    inc_y = goal.y -current_y

    angle_to_goal = atan2(inc_y, inc_x)
    print(angle_to_goal)

    if(theta-angle_to_goal > 0.1):
        speed.angular.z=-0.2
        speed.linear.x=0
        print("positive_difference",theta-angle_to_goal)

    elif(theta-angle_to_goal < -0.1):
        speed.angular.z=0.2
        speed.linear.x=0
        print("negative_difference",theta-angle_to_goal)

    else:
        speed.angular.z=0
        speed.linear.x =0.5
    
    if(inc_x <0.01 and inc_y<0.01):
        speed.linear.x=0
        speed.angular.z=0

    pub.publish(speed)
    
    
def obstacle_avoid():
    
  if(us_msg.r1 < 2 ):
        if(us_msg.r2 < 1.5 or us_msg.r3 < 1 ):          
                speed.linear.x = 0
                speed.angular.z = -0.5
                
                print("go straight of left case ")
        else:
               speed.linear.x = 0
               speed.angular.z =0.5
        
               print("go straight of right side ")
  else:
             speed.linear.x = 0.6
             speed.angular.z = 0
            
             print("left side rotate ")

  pub.publish(speed)
  




