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
target_x = 0
target_y = 0
dist_front=0


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
    #print('coordinates is',x_dist,y_dist)
    # print("angle is ",yaw_)

def clbk_laser(msg):
    global dist_front 
    region = {
	'p' : msg.ranges[:],
    }
    dist_front=msg.ranges[179]
    # region['p'][0] represents the 0 degree and 0the value start from back and continues in anti-clockwise direction
    # for i in range(360):
	# print region['p'][i]

flag=0

def linear_pid(orient):
    global limcoord, G, W, x_dist, y_dist, flag,target_x,target_y

    msg=Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    if(orient==0):
        if(flag==0):
            x_ind,y_ind = conversion()
            desired_x_ind = x_ind
            desired_y_ind = y_ind + 1
            if(desired_x_ind>15 or desired_y_ind>15 or desired_x_ind<0 or desired_y_ind<0):
                desired_x_ind = x_ind
                desired_y_ind = y_ind
            target_x = limcoord - (desired_x_ind*2+1)*G/2 - desired_x_ind*W
            target_y = limcoord - (desired_y_ind*2+1)*G/2 - desired_y_ind*W
            print("target",target_x,target_y)
            flag=1
        error = y_dist - target_y
        msg.linear.x = 0.8*error
        msg.angular.z = 0
        if abs(msg.linear.x) < 0.05:
            msg.linear.x = 0
            flag= 0
        pub.publish(msg)
        print("vel: ",msg.linear.x)
        print("error: ",error)
        print("current coordinate ",x_dist,y_dist)
        
    elif(orient==1):
        if(flag==0):
            x_ind,y_ind = conversion()
            desired_x_ind = x_ind + 1
            desired_y_ind = y_ind 
            if(desired_x_ind>15 or desired_y_ind>15 or desired_x_ind<0 or desired_y_ind<0):
                desired_x_ind = x_ind
                desired_y_ind = y_ind
            target_x = limcoord - (desired_x_ind*2+1)*G/2 - desired_x_ind*W
            target_y = limcoord - (desired_y_ind*2+1)*G/2 - desired_y_ind*W
            flag=1
        error = target_x - x_dist
        msg.linear.x = 0.8*error
        msg.angular.z = 0
        if abs(msg.linear.x) < 0.05:
            msg.linear.x = 0
            flag= 0
        pub.publish(msg)
        
    elif(orient==2):
        if(flag==0):
            x_ind,y_ind = conversion()
            desired_x_ind = x_ind
            desired_y_ind = y_ind - 1
            if(desired_x_ind>15 or desired_y_ind>15 or desired_x_ind<0 or desired_y_ind<0):
                desired_x_ind = x_ind
                desired_y_ind = y_ind
            target_x = limcoord - (desired_x_ind*2+1)*G/2 - desired_x_ind*W
            target_y = limcoord - (desired_y_ind*2+1)*G/2 - desired_y_ind*W
            flag=1
        error = y_dist - target_y
        msg.linear.x = 0.8*error
        msg.angular.z = 0
        if abs(msg.linear.x) < 0.05:
            msg.linear.x = 0
            flag= 0
        pub.publish(msg)
        
    elif(orient==3): 
        if(flag==0):
            x_ind,y_ind = conversion()
            desired_x_ind = x_ind - 1
            desired_y_ind = y_ind 
            if(desired_x_ind>15 or desired_y_ind>15 or desired_x_ind<0 or desired_y_ind<0):
                desired_x_ind = x_ind
                desired_y_ind = y_ind
            target_x = limcoord - (desired_x_ind*2+1)*G/2 - desired_x_ind*W
            target_y = limcoord - (desired_y_ind*2+1)*G/2 - desired_y_ind*W
            flag=1
        error = target_x - x_dist
        msg.linear.x = 0.8*error
        msg.angular.z = 0
        if abs(msg.linear.x) < 0.05:
            msg.linear.x = 0
            flag= 0
        pub.publish(msg)


def conversion():
    global x_dist,y_dist,limcoord,G,W
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

flag=0

def distance_maintainer_right():
        global dist_front
        p=2
        direction=-1
        wall_dist=0.075
        rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
        # dist_front=self.distan(333)
        
        max_speed=0.1

        # Determine values for PD control of distance and P control of angle
        for i in range(0,180):
            if(i==0):
                min_index=i
            if self.distan(i) < self.distan(min_index):
                min_index = i
        angle_min = math.radians((333-min_index)*(240/666))
        dist_min = self.distan(min_index)
        e = (dist_min - wall_dist)
        if dist_front < wall_dist:
            velocity1 = 0
        elif dist_front < wall_dist*2:
            velocity1 = 0.5*max_speed
        elif abs(angle_min) > 1.75:
            velocity1 = 0.4*max_speed
        else:
            velocity1 = max_speed
        # velocity1=6
        omega1 = max(min(direction*(p*e) + (angle_min-((math.pi)/2)*direction), 1.5), -1.5)

















def main():
    global yaw_,x_dist,y_dist,flag
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)   
    rospy.init_node('cmd_robot', anonymous=True)
    rate = rospy.Rate(0.4) # 40hz

    while not rospy.is_shutdown():
        # rospy.Subscriber('/odom', Odometry, clbk_odom) 
        # conversion()
        linear_pid(0)
        rate.sleep()


if __name__ == '__main__':
    main()


