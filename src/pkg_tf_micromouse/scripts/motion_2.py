#!/usr/bin/env python

#from sre_parse import GLOBAL_FLAGS
from turtle import goto
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from math import degrees, pow,sqrt,atan2
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from package1234.msg import dest
import sys
import math
import time
import random
import numpy as np  
from numpy import ma
from sensor_msgs.msg import LaserScan



class sambit_controller():
    def __init__(self):
        rospy.init_node("go_to_goal_controller")
        print("enter init")
       
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist , queue_size = 1)
        self.theta_pub = rospy.Publisher("/theta" , Float32 , queue_size = 1)
        rospy.Subscriber('/odom', Odometry , self.odom_callback)
        rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, self.clbk_laser)
        rospy.Subscriber('/dest' , dest , self.goal_pose_callback , queue_size = 1 , buff_size = 40)
        rospy.Subscriber('/run_number', Int32 , self.run_number_callback)
        self.ack_pub = rospy.Publisher('/acknowledge', Float32 , queue_size=1)

        self.prev_goal = [0.0 , 0.0]
        self.goal = [0 , 0]
        self.current_pose = [-1.35 , 1.23 , 0.0]  #x,y,theta
        self.current_orientation_euler = [0.0 , 0.0 , 0.0 ]
        self.current_orientation_quaternion = [0.0 , 0.0 , 0.0 , 0.0]

        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0

        self.sample_time = 0.05
        self.previous_theta = 0.0

        self.reached_x = False
        self.reached_y = False

        self.publishing_rate = rospy.Rate(20)

        self.max_speed_y = 0.15
        self.max_speed_x = 0.15
        self.max_speed_scaled_x = self.max_speed_x*10
        self.max_speed_scaled_y = self.max_speed_y*10
        self.min_thresh = 0.005 #if distance to goal is btw 0.05 and 0.01, then p controller is used to prevent overshoot
        self.max_thresh = 0.05  #min distance upto which the speed given to the bot is 0.4
        self.init_current_pose = [0.0 , 0.0 ,0.0]

        self.run_number = 1

        self.flag=1
        self.flag2=1
        self.flag3=1

        self.C1=0
        self.C2=0
        self.direction=0

        self.change_check_goal=0


        ###wall_follower constant#
        self.mode=1                      # 1 for wall on the left side of the robot (-1 for the right side)
        self.wall_dist=0.07
        self.inf=0.3
        self.e = 0                       # Diference between current wall measurements and previous one
        self.angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
        self.dist_front = 0              # Measured front distance
        self.diff_e = 0                  # Difference between current error and previous one
        self.dist_min = 0                # Minimum measured distance
        self.angle=1

        self.max_left=1
        self.max_right=0
        self.a=0
        self.min_left=0
        self.min_right=0

        self.regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
        }





    def run_number_callback(self, msg):
        self.run_number = msg.data

    def odom_callback(self,msg):
        # print("enter odom callback")
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y

        self.current_orientation_quaternion[0] = msg.pose.pose.orientation.x
        self.current_orientation_quaternion[1] = msg.pose.pose.orientation.y
        self.current_orientation_quaternion[2] = msg.pose.pose.orientation.z
        self.current_orientation_quaternion[3] = msg.pose.pose.orientation.w
        (self.current_orientation_euler[1] , self.current_orientation_euler[0] , self.current_orientation_euler[2]) = euler_from_quaternion([self.current_orientation_quaternion[0] , self.current_orientation_quaternion[1] , self.current_orientation_quaternion[2] , self.current_orientation_quaternion[3]])
        self.current_pose[2] = self.current_orientation_euler[2]
        
        

    def goal_theta(self):
        self.goal_yaw = atan2(self.goal[1] - self.current_pose[1], self.goal[0] - self.current_pose[0])  
        # print("goal_YAW", self.goal_yaw)
        return self.goal_yaw

        
    def distance_from_goal(self , pose_x , pose_y , goal_x , goal_y):
        distance = sqrt(pow((goal_x - pose_x), 2) + pow((goal_y - pose_y), 2))
        return distance


    def linear(self):
        # print("linear goal is ",self.goal)
        self.vel.linear.x=abs(0.4*((((self.current_pose[1] - self.goal[1] )**2) + ((self.current_pose[0]-self.goal[0])**2) )**0.5))
        distance=((((self.current_pose[1] - self.goal[1] )**2) + ((self.current_pose[0]-self.goal[0])**2) )**0.5)

        if(self.vel.linear.x>0.1):
            self.vel.linear.x=0.1
        # print("current linear velocity is ",self.vel.linear.x)

        if(abs(self.vel.linear.x)<0.0085):
        # if(distance<0.04):
            print("enter critical linear pid")
            self.vel.linear.x=0
            self.flag2=0
            self.flag3=1
            self.flag=1

    # def change_goal(self):
    #     self.prev_goal[0]=self.goal[0]
    #     self.prev_goal[1]=self.goal[1]
    #     if(self.goal==self.prev_goal):
    #         a=0  #no goal change
    #     else:
    #         a=1  #goal change
    #     return a
        

               
    # def angular(self):
    #     a=self.goal_theta()
    #     print("tan inverse is ",a)
    #     print("current pose is ",self.current_pose)
    #     error =  a
    #     print("error is ",error)
    #     # if -math.pi<self.current_pose[2]<-0.75*math.pi:
    #     #     self.current_pose[2]= self.current_pose[2] +2*math.pi
    #     kp= 0.35
    #     w = kp*error
    #     if(w>0.2):
    #         w=0.2
    #     elif(w<-0.2):
    #         w=-0.2

    #     if abs(w)<0.03:
    #         w=0
    #         # flag2=1
    #         # flag =1
    #     self.vel.angular.z = w

    def clbk_laser(self,msg):
        """
        Read sensor messagens, and determine distance to each region. 
        Manipulates the values measured by the sensor.
        Callback function for the subscription to the published Laser Scan values.
        """
        ##checking direction mode
        # self.max_right=mamsg.ranges[10:40])
        # self.max_left=max(msg.ranges[319,349])

        for i in range(10,41):
            if(i==10):
                self.max_right=msg.ranges[10]
            if(self.max_right<msg.ranges[i]):
                self.max_right=msg.ranges[i]

        for i in range(319,350):
            if(i==319):
                self.max_left=msg.ranges[319]
            if(self.max_left<msg.ranges[i]):
                self.max_left=msg.ranges[i]
        


        if(self.max_left>self.max_right):
            # print("follow right wall")
            self.mode=-1  #follow right wall
        else:
            # print("follow left wall")
            self.mode=1   #follow left wall


        size = len(msg.ranges)
        min_index = size*(self.mode+1)/4
        max_index = size*(self.mode+3)/4
        
        # Determine values for PD control of distance and P control of angle
        for i in range(min_index, max_index):
            if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.02:
                min_index = i
        self.angle_min = (min_index-size/2)*msg.angle_increment
        self.dist_min = msg.ranges[min_index]
        self.dist_front = msg.ranges[size/2]
        self.diff_e = min((self.dist_min - self.wall_dist) - self.e, 100)
        self.e = min(self.dist_min - self.wall_dist, 100)

        # Determination of minimum distances in each region
        self.regions_ = {
            
            'right': min(min(msg.ranges[0:71]), self.inf),
            'fright':  min(min(msg.ranges[72:143]), self.inf),
            'front':  min(min(msg.ranges[144:215]), self.inf),
            'fleft':   min(min(msg.ranges[216:287]), self.inf),
            'left':   min(min(msg.ranges[288:359]), self.inf)
        }

        self.min_left=min(msg.ranges[359-91:359-30])
        self.min_right=min(msg.ranges[30:91])


        #rospy.loginfo(regions_)
        
        # take_action()

    def following_wall(self):
        # print("enter following_wall")
        """
        PD control for the wall following state. 
        Returns:
                Twist(): msg with angular and linear velocities to be published
                        msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                        msg.angular.z -> PD controller response
        """
        p=15
        d=0
        angular_velocity = max(min(self.mode*(p*self.e+d*self.diff_e) + self.angle*(self.angle_min-((math.pi)/2)*self.mode), 1), -1)
        #print 'Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x)
        return angular_velocity






    def turn(self,angle):
        
        if -math.pi<self.current_pose[2]<-0.75*math.pi:
            self.current_pose[2]= self.current_pose[2] +2*math.pi
        error = -self.current_pose[2] + math.radians(angle)
        # print("yaw angle is ",math.degrees(self.current_pose[2]))
        # print("error in degrees ",math.degrees(error))
        kp= 0.35
        w = kp*error

        if(w>0.2):
            w=0.2
        elif(w<-0.2):
            w=-0.2

        # print("value of w is ",w)
        if abs(w)<0.03:
            print("enter threshold angular w")
            w=0
            self.flag2=1
            self.flag =1
        self.vel.angular.z = w
        self.vel.linear.x=0
        self.vel.linear.y=0



    def control(self):
        print("current goal is ",self.goal)
        if(self.goal!=[0,0]):
            # print("checking changing goal")
            # self.change_check_goal=self.change_goal()
            print("self change check goal value is ",self.change_check_goal)
            print("previous goal is ",self.prev_goal)
            if(self.change_check_goal==1):
                print("goal changed successfully")
                #goal change
                self.vel.linear.x=0
                self.vel.angular.z=0
                self.vel_pub.publish(self.vel)
                self.flag3=1
                self.flag=1
                self.flag2=0

        if(self.flag3==1):
            # print("enter flag3")
            # print("goal is ",self.goal)
            # print("current pose is ",self.current_pose)
            delx = self.goal[0]-self.current_pose[0]
            dely = self.goal[1]-self.current_pose[1]

            # print("delx is ",delx)
            # print("del y is ",dely)

            if(abs(delx)<0.16):
                self.C1=0
            elif(delx>0.16):
                self.C1=1
            elif(delx<-0.16):
                self.C1=-1

            if(abs(dely)<0.16):
                self.C2=0
            elif(dely>0.16):
                self.C2=1
            elif(dely<-0.16):
                self.C2=-1
            print("C1 and C2 is ",self.C1,self.C2)
            self.flag3=0
            self.flag=0
            self.flag2=0

        # print("C1 and C2 is ",self.C1,self.C2)        

        if self.flag == 0:
            print("enter turn")
            if self.C1 == 0 and self.C2==-1:
                print('0')
                self.turn(0)  
            elif self.C1 == 1 and self.C2==0:
                # self.turn(-90)
                # print("-90")
                self.turn(90)
                print("90")
            elif self.C1 == 0 and self.C2==1:
                self.turn(180)
                print("180")
            elif self.C1 == -1 and self.C2==0:
                self.turn(-90)
                print("-90")
            elif self.C1==0 and self.C2==0:
                print("c1 and c2 both are zero")
                self.turn(0)
            else:
                self.flag3=1
                self.flag=1
             
        elif(self.flag2==1):
            print("enter linear + angular")
            self.linear()
            if(self.min_right>0.09 and self.min_left>0.09):
                print("no wall following")
                self.vel.angular.z=0
            else:
                self.vel.angular.z=self.following_wall()
            # self.angular()

        # print("publisher below")
        self.vel_pub.publish(self.vel)
        self.theta_pub.publish(self.current_pose[2])
        # print("x:" , self.current_pose[0])
        # print("y:" , self.current_pose[1])
        # print("theta:" , self.current_pose[2])

    def goal_pose_callback(self,msg):
        self.prev_goal[0]=self.goal[0]
        self.prev_goal[1]=self.goal[1]

        self.goal[0] = msg.dest_x_coordinate
        self.goal[1] = msg.dest_y_coordinate

        #self.change_check_goal=self.change_goal()
        if(self.goal==self.prev_goal):
            self.a=0  #goal same
        else:
            self.a=1  #goal change
        self.change_check_goal=self.a
        print('Prev Goal location:' ,self.prev_goal[0],', ', self.prev_goal[1])
        print('Goal location:' ,self.goal[0],', ', self.goal[1])
        print('self change check goal value is ',self.change_check_goal)
            
   
if __name__ == "__main__":
    rospy.sleep(1.5)
    y = sambit_controller()
    # args = rospy.myargv(argv=sys.argv)
    # print("pre exit")
    # if len(args) != 3:
    #     print("Incorrect number of arguments")
    #     sys.exit()
    # print("post exit")
    # yo.goal[0] = float(args[1])
    # yo.goal[1] = float(args[2])

    max_speed = 0.02
    while not rospy.is_shutdown():
        # print("Current speed: %f" % yo.max_speed_x)
        # yo.max_speed_x = max_speed + 0.03*(yo.run_number//2)
        # yo.max_speed_y = max_speed + 0.03*(yo.run_number//2)
        # yo.goal_pose_callback
        y.control()
        y.publishing_rate.sleep()
    






