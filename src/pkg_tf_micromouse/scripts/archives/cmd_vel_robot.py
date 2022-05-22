#!/usr/bin/env python
import rospy
from rospy.topics import Publisher
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math
import numpy as np

x_dist = 0
y_dist = 0
yaw_=0
def clbk_odom(msg):
    global x_dist
    global y_dist
    global yaw_
    # position
    position_ = msg.pose.pose.position
    # gives x and y distance of the bot
    x_dist = position_.x
    y_dist = position_.y
    
    # yaw
    # convert quaternions to euler angles, only extracting yaw angle for the robot
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    yaw_ = euler[2]
    # print(yaw_)

def clbk_laser(msg):
    region = {
	'p' : msg.ranges[:],
    }
    # region['p'][0] represents the 0 degree and 0the value start from back and continues in anti-clockwise direction
    for i in range(360):
	    print region['p'][i]

def check():
    # global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg=Twist()
    msg.linear.x=0
    msg.angular.z=1
    pub.publish(msg)

visited=np.full((16,16),0)
def conversion():
    global x_dist,y_dist,G
    x_coordinate = x_dist
    y_coordinate = y_dist
    index_x,index_y=0,0

    limcoord= 1.436
    G = 0.170125
    W = 0.01
    if limcoord>=y_coordinate>limcoord-1*G-(0*2+1)*W/2:
        index_y= 0
    elif limcoord-1*G-(0*2+1)*W/2>=y_coordinate>limcoord-2*G-(1*2+1)*W/2:
        index_y= 1
    elif limcoord-2*G-(1*2+1)*W/2>=y_coordinate>limcoord-3*G-(2*2+1)*W/2:
        index_y= 2
    elif limcoord-3*G-(2*2+1)*W/2>=y_coordinate>limcoord-4*G-(3*2+1)*W/2:
        index_y= 3
    elif limcoord-4*G-(3*2+1)*W/2>=y_coordinate>limcoord-5*G-(4*2+1)*W/2:
        index_y= 4
    elif limcoord-5*G-(4*2+1)*W/2>=y_coordinate>limcoord-6*G-(5*2+1)*W/2:
        index_y= 5  
    elif limcoord-6*G-(5*2+1)*W/2>=y_coordinate>limcoord-7*G-(6*2+1)*W/2: 
        index_y= 6  
    elif limcoord-7*G-(6*2+1)*W/2>=y_coordinate>limcoord-8*G-(7*2+1)*W/2:
        index_y= 7
    elif limcoord-8*G-(7*2+1)*W/2>=y_coordinate>limcoord-9*G-(8*2+1)*W/2:
        index_y= 8
    elif limcoord-9*G-(8*2+1)*W/2>=y_coordinate>limcoord-10*G-(9*2+1)*W/2:
        index_y= 9
    elif limcoord-10*G-(9*2+1)*W/2>=y_coordinate>limcoord-11*G-(10*2+1)*W/2:
        index_y= 10
    elif limcoord-11*G-(10*2+1)*W/2>=y_coordinate>limcoord-12*G-(11*2+1)*W/2:
        index_y= 11
    elif limcoord-12*G-(11*2+1)*W/2>=y_coordinate>limcoord-13*G-(12*2+1)*W/2:
        index_y= 12
    elif limcoord-13*G-(12*2+1)*W/2>=y_coordinate>limcoord-14*G-(13*2+1)*W/2:
        index_y= 13
    elif limcoord-14*G-(13*2+1)*W/2>=y_coordinate>limcoord-15*G-(14*2+1)*W/2:
        index_y= 14
    elif limcoord-15*G-(14*2+1)*W/2>=y_coordinate>limcoord-16*G-(15*2)*W/2:
        index_y= 15

    if limcoord>=x_coordinate>limcoord-1*G-(0*2+1)*W/2:
        index_x= 0
    elif limcoord-1*G-(0*2+1)*W/2>=x_coordinate>limcoord-2*G-(1*2+1)*W/2:
        index_x= 1
    elif limcoord-2*G-(1*2+1)*W/2>=x_coordinate>limcoord-3*G-(2*2+1)*W/2:
        index_x= 2
    elif limcoord-3*G-(2*2+1)*W/2>=x_coordinate>limcoord-4*G-(3*2+1)*W/2:
        index_x= 3
    elif limcoord-4*G-(3*2+1)*W/2>=x_coordinate>limcoord-5*G-(4*2+1)*W/2:
        index_x= 4
    elif limcoord-5*G-(4*2+1)*W/2>=x_coordinate>limcoord-6*G-(5*2+1)*W/2:
        index_x= 5    
    elif limcoord-6*G-(5*2+1)*W/2>=x_coordinate>limcoord-7*G-(6*2+1)*W/2: 
        index_x= 6   
    elif limcoord-7*G-(6*2+1)*W/2>=x_coordinate>limcoord-8*G-(7*2+1)*W/2:
        index_x= 7
    elif limcoord-8*G-(7*2+1)*W/2>=x_coordinate>limcoord-9*G-(8*2+1)*W/2:
        index_x= 8
    elif limcoord-9*G-(8*2+1)*W/2>=x_coordinate>limcoord-10*G-(9*2+1)*W/2:
        index_x= 9
    elif limcoord-10*G-(9*2+1)*W/2>=x_coordinate>limcoord-11*G-(10*2+1)*W/2:
        index_x= 10
    elif limcoord-11*G-(10*2+1)*W/2>=x_coordinate>limcoord-12*G-(11*2+1)*W/2:
        index_x= 11
    elif limcoord-12*G-(11*2+1)*W/2>=x_coordinate>limcoord-13*G-(12*2+1)*W/2:
        index_x= 12
    elif limcoord-13*G-(12*2+1)*W/2>=x_coordinate>limcoord-14*G-(13*2+1)*W/2:
        index_x= 13
    elif limcoord-14*G-(13*2+1)*W/2>=x_coordinate>limcoord-15*G-(14*2+1)*W/2:
        index_x= 14
    elif limcoord-15*G-(14*2+1)*W/2>=x_coordinate>limcoord-16*G-(15*2)*W/2:
        index_x= 15
    # print("indices are ",index_x,index_y)
    return index_x, index_y


prev_index_x,prev_index_y=0,0
def update_index():
    global prev_index_x,prev_index_y,visited
    index_x,index_y=conversion()
    print(index_x,index_y,"indexes are ")
    if(abs(index_x-prev_index_x)>=1 or abs(index_y-prev_index_y)>=1):   
        visited[15-index_y][index_x]=1+visited[15-index_y][index_x]
    prev_index_x=index_x
    prev_index_y=index_y
    if(visited[15-index_y][index_x]>2):
        visited[15-index_y][index_x]=2

def main():
    global yaw_,visited
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) 
    # sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)  
    rospy.init_node('cmd_robot', anonymous=True)
    rate = rospy.Rate(50) # 40hz

    while not rospy.is_shutdown():
        update_index()
        msg1 = Twist()
	# #positive speed_z value represents clockwise angular velocity of the bot and positive speed_x value represents forward linear velocity of the robot
    #     speed_z = 0
        speed_x = 0
        msg1.linear.x = speed_x
        msg1.angular.z = 0.5
        pub.publish(msg1)
        print(visited)
        # check()
        # print(math.degrees(yaw_))
        rate.sleep()


if __name__ == '__main__':
    main()
