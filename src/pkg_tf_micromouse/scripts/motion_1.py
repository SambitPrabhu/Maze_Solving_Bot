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



class gtg_controller():
    def __init__(self):
        rospy.init_node("go_to_goal_controller")
        print("enter init")
       
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist , queue_size = 1)
        self.theta_pub = rospy.Publisher("/theta" , Float32 , queue_size = 1)
        rospy.Subscriber('/odom', Odometry , self.odom_callback)
        # rospy.Subscriber('/dest' , dest , self.goal_pose_callback , queue_size = 1 , buff_size = 40)
        rospy.Subscriber('/run_number', Int32 , self.run_number_callback)
        self.ack_pub = rospy.Publisher('/acknowledge', Float32 , queue_size=1)

        #self.prev_goal = [0.0 , 0.0]
        self.goal = [0.20 , -1.35]
        self.current_pose = [-1.35 , -1.35 , 0.0]  #x,y,theta
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
        
        self.vel.linear.x=abs(0.9*((((self.current_pose[1] - self.goal[1] )**2) + ((self.current_pose[0]-self.goal[0])**2) )**0.5))
        self.vel.angular.z=0

        if(self.vel.linear.x>0.2):
            self.vel.linear.x=0.2

        if(abs(self.vel.linear.x)<0.008):
            print("enter critical linear pid")
            self.vel.linear.x=0
            self.flag2=0
            self.flag3=1
            self.flag=1


        # if (self.goal[1]- self.current_pose[1])<0.161:
        #     if self.goal[0] - self.current_pose[0] >= 0:
        #         if self.goal[0] - self.current_pose[0] >= self.max_thresh:
        #             self.vel.linear.x = self.max_speed_x #0.1 may change, or to cover the path faster we can put constant 0.4 and when it reaches goal we can make it 0
        #         #print("away from goal in X_111111111111111111")
        #         elif (self.goal[0] - self.current_pose[0]) <= self.max_thresh and (self.goal[0] - self.current_pose[0]) >= self.min_thresh:
        #             self.vel.linear.x = self.max_speed_scaled_x * (self.goal[0] - self.current_pose[0])
        #         #print("getting close in X_11111111111")
        #         else:    
        #             self.vel.linear.x = 0.0
        #             self.flag= 0
        #             self.flag2=0
        #         #print("Give next goal in X_111111")
               
        #     else:
        #         if self.goal[0] - self.current_pose[0] <= (-1 * self.max_thresh):
        #             self.vel.linear.x = (-1 * self.max_speed_x) #0.1 may change, or to cover the path faster we can put constant 0.4 and when it reaches goal we can make it 0
        #         #print("away from goal in X_2222222222222")
        #         elif (self.goal[0] - self.current_pose[0]) >= (-1 * self.max_thresh) and (self.goal[0] - self.current_pose[0]) <= (-1 * self.min_thresh):
        #             self.vel.linear.x = self.max_speed_scaled_x * (self.goal[0] - self.current_pose[0])
        #         #print("getting close in X_222222222222")
        #         else:    
        #             self.vel.linear.x = 0.0
        #             self.flag= 0
        #             self.flag2=0
        #         #print("Give next goal in X_2222222222")
                    
        # elif (self.goal[0]- self.current_pose[0])<0.161:
        #     if self.goal[1] - self.current_pose[1] >= 0:
        #         if self.goal[1] - self.current_pose[1] > self.max_thresh:
        #             self.vel.linear.x = -self.max_speed_y #0.1 may change, or to cover the path faster we can put constant 0.4 and when it reaches goal we can make it 0
        #         elif (self.goal[1] - self.current_pose[1]) <= self.max_thresh and (self.goal[1] - self.current_pose[1]) >= self.min_thresh:
        #             self.vel.linear.x = -self.max_speed_scaled_y * ((self.goal[1] - self.current_pose[1]))
        #         else:    
        #             self.vel.linear.y = -0.0  
        #             self.flag= 0
        #             self.flag2=0
                
                
        #     else:
        #         if self.goal[1] - self.current_pose[1] < (-1 * self.max_thresh):
        #             self.vel.linear.x = self.max_speed_y #0.1 may change, or to cover the path faster we can put constant 0.4 and when it reaches goal we can make it 0
                
        #         elif (self.goal[1] - self.current_pose[1]) >= (-1 * self.max_thresh) and (self.goal[1] - self.current_pose[1]) <= (-1 * self.min_thresh):
        #             self.vel.linear.x = -self.max_speed_scaled_y * ((self.goal[1] - self.current_pose[1]))
        #         else:    
        #             self.vel.linear.x = -0.0 
        #             self.flag= 0
        #             self.flag2=0



               
    def angular(self):
        a=self.goal_theta()
        print("tan inverse is ",a)
        print("current pose is ",self.current_pose)
        error =  -a
        print("error is ",error)
        # if -math.pi<self.current_pose[2]<-0.75*math.pi:
        #     self.current_pose[2]= self.current_pose[2] +2*math.pi
        kp= 0.35
        w = kp*error
        if(w>0.2):
            w=0.2
        elif(w<-0.2):
            w=-0.2

        if abs(w)<0.03:
            w=0
            # flag2=1
            # flag =1
        self.vel.angular.z = w

    def turn(self,angle):
        
        if -math.pi<self.current_pose[2]<-0.75*math.pi:
            self.current_pose[2]= self.current_pose[2] +2*math.pi
        error = -self.current_pose[2] + math.radians(angle)
        print("yaw angle is ",math.degrees(self.current_pose[2]))
        print("error in degrees ",math.degrees(error))
        kp= 0.35
        w = kp*error

        if(w>0.2):
            w=0.2
        elif(w<-0.2):
            w=-0.2

        print("value of w is ",w)
        if abs(w)<0.03:
            print("enter threshold angular w")
            w=0
            self.flag2=1
            self.flag =1
        self.vel.angular.z = w
        self.vel.linear.x=0
        self.vel.linear.y=0

    def control(self):
        if(self.flag3==1):
            print("enter flag3")
            print("goal is ",self.goal)
            print("current pose is ",self.current_pose)
            delx = self.goal[0]-self.current_pose[0]
            dely = self.goal[1]-self.current_pose[1]

            print("delx is ",delx)
            print("del y is ",dely)

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

        

        if self.flag == 0:
            print("enter turn")
            if self.C1 == 0 and self.C2==-1:
                self.turn(0)
                print('0')
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
             
        elif(self.flag2==1):
            print("enter linear + angular")
            self.linear()
            self.angular()

        # print("publisher below")
        self.vel_pub.publish(self.vel)
        self.theta_pub.publish(self.current_pose[2])
        # print("x:" , self.current_pose[0])
        # print("y:" , self.current_pose[1])
        # print("theta:" , self.current_pose[2])

    # def goal_pose_callback(self,msg):
    #     self.goal[0] = -1.35207
    #     self.goal[1] = -0.1150
        
    #     # print("call_back recieved")
    #     print("Goal location:" ,self.goal[0],", ", self.goal[1])
        
    #     # print("ready to go to desination")
            
   
if __name__ == "__main__":
    yo = gtg_controller()
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
        yo.control()
        yo.publishing_rate.sleep()
    





