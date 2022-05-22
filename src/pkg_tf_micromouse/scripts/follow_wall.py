#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

e=0

angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
dist_front = 0              # Measured front distance
diff_e = 0                  # Difference between current error and previous one
dist_min = 0  
max_speed = 0.1
direction = -1
p = 15                      # Proportional constant for controller  
d = 0  
angle = 1



inf=0.18
active_ = False
wall_dist=0.085
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res
size=0

def clbk_laser(msg):
    global regions_,inf,wall_dist,e,angle_min,dist_front,diff_e,dist_front
    size = len(msg.ranges)

    direction=-1


    min_index = size*(direction+1)/4
    max_index = size*(direction+3)/4
    
    # Determine values for PD control of distance and P control of angle
    for i in range(min_index, max_index):
        if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.04:
            min_index = i
    angle_min = (min_index-size/2)*msg.angle_increment
    dist_min = msg.ranges[min_index]
    dist_front = msg.ranges[size/2]
    diff_e = min((dist_min - wall_dist) - e, 100)
    e = min(dist_min - wall_dist, 100)




    regions_ = {
        'right': min(min(msg.ranges[0:71]), inf),
        'fright':  min(min(msg.ranges[72:143]), inf),
        'front':  min(min(msg.ranges[144:215]), inf),
        'fleft':   min(min(msg.ranges[216:287]), inf),
        'left':   min(min(msg.ranges[288:359]), inf)
    }
    
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    d = 0.1
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        # print(state_description)
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        # print(state_description)
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        # print(state_description)
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        # print(state_description)
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        # print(state_description)
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        # print(state_description)
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        # print(state_description)
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        # print(state_description)
        change_state(0)
    else:
        state_description = 'unknown case'
        # print(state_description)
        rospy.loginfo(regions)

def find_wall():
    # print("enter following_wall")
    """
    PD control for the wall following state. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                    msg.angular.z -> PD controller response
    """
    global wall_dist, max_speed, direction, p, d, angle, dist_min, dist_front, e, diff_e, angle_min
    msg = Twist()
    if dist_front < wall_dist:
        # print("enter 1")
        msg.linear.x = 0
    elif dist_front < wall_dist*2:
        # print("enter 2")
        msg.linear.x = 0.5*max_speed
    elif abs(angle_min) > 1.75:
        # print("enter 3")
        msg.linear.x = 0.4*max_speed
    else:
        # print("enter 4")
        msg.linear.x = max_speed
    msg.angular.z = max(min(direction*(p*e+d*diff_e) + angle*(angle_min-((math.pi)/2)*direction), 2.5), -2.5)
    #print 'Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x)
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.3
    return msg

def follow_the_wall():
    global regions_
    msg = Twist()
    msg.linear.x = 0.1
    return msg

def main():
    
    global pub_, active_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    
    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # print("enter the wall following")
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        if state_ == 0:
            # print("find wall")
            msg = find_wall()
        elif state_ == 1:
            # print("turn left")
            msg = turn_left()
        elif state_ == 2:
            # print("follow wall")
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()
