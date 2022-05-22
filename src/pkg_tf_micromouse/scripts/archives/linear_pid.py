#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math

x_dist = 0
y_dist = 0
yaw_=0
x_final=0
y_final=0


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
    
    # yaw_ = math.degrees(quaternion[2])
    yaw_ = euler[2]
    print(y_dist)
    # print("angle is ",yaw_)

def clbk_laser(msg):
    region = {
	'p' : msg.ranges[:],
    }
    # region['p'][0] represents the 0 degree and 0the value start from back and continues in anti-clockwise direction
    for i in range(360):
	print region['p'][i]
flag=0
def main():
    global yaw_,x_dist,y_dist,flag,x_final,y_final
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)   
    rospy.init_node('cmd_robot', anonymous=True)
    rate = rospy.Rate(1) # 40hz

    while not rospy.is_shutdown():
        msg1 = Twist()
        rospy.Subscriber('/odom', Odometry, clbk_odom) 
        if(flag==0):
            initial_position=y_dist
            flag=1
	# #positive speed_z value represents clockwise angular velocity of the bot and positive speed_x value represents forward linear velocity of the robot
        error=(y_dist)
        speed_z = 0.1*error
        speed_x = error
        print("speed of x is ",speed_x)
        if(speed_x>0.1):
            print("enter max speed")
            speed_x=0.1
        msg1.linear.x = speed_x
        msg1.angular.z = 0
        pub.publish(msg1)
        rate.sleep()


if __name__ == '__main__':
    main()


s