#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

lidar_range=[0]*360

def clbk_laser(msg):
    global lidar_range
    # 720 / 5 = 144
    # print(min(msg.ranges[0],0.09),"index 0 reading")
    # print(min(msg.ranges[179],0.09),"index 179 reading")
    # print(min(msg.ranges[359],0.09),"index 359 reading")
    for i in range(0,360):
        lidar_range[i]=min(msg.ranges[i],0.09)

# def distance_lidar(msg):
#     return msg.ranges[i]

def main():
    global lidar_range
    rospy.init_node('reading_laser')
    sub=rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():      
        # rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
        # sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, distance_lidar(180))
        print(lidar_range[0])
        


if __name__ == '__main__':
    main()
