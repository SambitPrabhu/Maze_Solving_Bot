#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math
import time
import random
import numpy as np  
from numpy import ma



##################################################################################


hz = 50                     # Cycle Frequency
loop_index = 0              # Number of sampling cycles
loop_index_outer_corner = 0 # Loop index when the outer corner is detected
loop_index_inner_corner = 0 # Loop index when the inner corner is detected
inf = 0.09                    # Limit to Laser sensor range in meters, all distances above this value are 
                            #      considered out of sensor range
wall_dist = 0.09           # Distance desired from the wall
max_speed = 0.1             # Maximum speed of the robot on meters/seconds
p = 15                      # Proportional constant for controller  
d = 0                       # Derivative constant for controller 
angle = 1                   # Proportional constant for angle controller (just simple P controller)
direction = 1              # 1 for wall on the left side of the robot (-1 for the right side)
e = 0                       # Diference between current wall measurements and previous one
angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
dist_front = 0              # Measured front distance
diff_e = 0                  # Difference between current error and previous one
dist_min = 0                # Minimum measured distance

# Time when the last outer corner; direction and inner corner were detected or changed.
last_outer_corner_detection_time = time.time()
last_change_direction_time = time.time()
last_inner_corner_detection_time = time.time()
rotating = 0 
pub_ = None
# Sensor regions
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
last_kinds_of_wall=[0, 0, 0, 0, 0]
index = 0

state_outer_inner=[0, 0, 0, 0]
index_state_outer_inner = 0

bool_outer_corner = 0
bool_inner_corner =0

last_vel = [random.uniform(0.1,0.3),  random.uniform(-0.3,0.3)]
wall_found =0

#Robot state machines
state_ = 0
state_dict_ = {
    0: 'random wandering',
    1: 'following wall',
    2: 'rotating'
}
lidar_range=[0]*360
def clbk_laser(msg):
    """
    Read sensor messagens, and determine distance to each region. 
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    global regions_, e, angle_min, dist_front, diff_e, direction, bool_outer_corner, bool_inner_corner, index, last_kinds_of_wall,lidar_range

    for i in range(360):
        lidar_range[i]=msg.ranges[i]
    



    size = len(msg.ranges)
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

    # Determination of minimum distances in each region
    regions_ = {
        
        'right': min(min(msg.ranges[0:71]), inf),
        'fright':  min(min(msg.ranges[72:143]), inf),
        'front':  min(min(msg.ranges[144:215]), inf),
        'fleft':   min(min(msg.ranges[216:287]), inf),
        'left':   min(min(msg.ranges[288:359]), inf)
    }
    #rospy.loginfo(regions_)

    # Detection of Outer and Inner corner
    bool_outer_corner = is_outer_corner()
    bool_inner_corner = is_inner_corner()
    if bool_outer_corner == 0 and bool_inner_corner == 0:
        last_kinds_of_wall[index]=0
    
    # Indexing for last five pattern detection
    # This is latter used for low pass filtering of the patterns
    index = index + 1 #5 samples recorded to asses if we are at the corner or not
    if index == len(last_kinds_of_wall):
        index = 0
        
    take_action()

def change_state(state):
    """
    Update machine state
    """
    global state_, state_dict_
    if state is not state_:
        #print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

def take_action():
    """
    Change state for the machine states in accordance with the active and inactive regions of the sensor.
            State 0 No wall found - all regions infinite - Random Wandering
            State 1 Wall found - Following Wall
            State 2 Pattern sequence reached - Rotating
    """
    global regions_, index, last_kinds_of_wall, index_state_outer_inner, state_outer_inner, loop_index, loop_index_outer_corner
    
    global wall_dist, max_speed, direction, p, d, angle, dist_min, wall_found, rotating, bool_outer_corner, bool_inner_corner

    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    # Patterns for rotating
    rotate_sequence_V1 = ['I', 'C', 'C', 'C']
    rotate_sequence_V2 = [0, 'C', 'C', 'C']
    rotate_sequence_W = ['I', 'C', 'I', 'C']

    if rotating == 1:
        state_description = 'case 2 - rotating'
        # change_state(2)
        change_state(1)
        if(regions['left'] < wall_dist or regions['right'] < wall_dist):
            rotating = 0
    elif regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['fleft'] == inf and regions['left'] == inf:
        state_description = 'case 0 - random wandering'
        # change_state(0)
        change_state(1)
    elif (loop_index == loop_index_outer_corner) and (rotate_sequence_V1 == state_outer_inner or rotate_sequence_V2 == state_outer_inner or rotate_sequence_W == state_outer_inner):
        state_description = 'case 2 - rotating'
        change_direction()
        state_outer_inner = [ 0, 0,  0, 'C']
        change_state(2)
    else:
        state_description = 'case 1 - following wall'
        change_state(1)

def random_wandering():
    """
    This function defines the linear.x and angular.z velocities for the random wandering of the robot.
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> [0.1, 0.3]
                    msg.angular.z -> [-1, 1]
    """
    global direction, last_vel
    msg = Twist()
    msg.linear.x = max(min( last_vel[0] + random.uniform(-0.01,0.01),0.3),0.1)
    msg.angular.z= max(min( last_vel[1] + random.uniform(-0.1,0.1),1),-1)
    if msg.angular.z == 1 or msg.angular.z == -1:
        msg.angular.z = 0
    last_vel[0] = msg.linear.x
    last_vel[1] = msg.angular.z
    return msg

def following_wall():
    #  print("enter following_wall")
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

def change_direction():
    """
    Toggle direction in which the robot will follow the wall
        1 for wall on the left side of the robot and -1 for the right side
    """
    global direction, last_change_direction, rotating
    print('Change direction!')
    elapsed_time = time.time() - last_change_direction_time # Elapsed time since last change direction
    if elapsed_time >= 20:
        last_change_direction = time.time()
        #direction = -direction # Wall in the other side now
        direction = 1
        rotating = 1

def rotate():
    """
    Rotation movement of the robot. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0m/s
                    msg.angular.z -> -2 or +2 rad/s
    """
    global direction
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = direction*2
    return msg


def is_outer_corner():
    """
    Assessment of outer corner in the wall. 
    If all the regions except for one of the back regions are infinite then we are in the presence of a possible corner.
    If all the elements in last_kinds_of_wall are 'C' and the last time a real corner was detected is superior or equal to 30 seconds:
        To state_outer_inner a 'C' is appended and 
        The time is restart.
    Returns:
            bool_outer_corner: 0 if it is not a outer corner; 1 if it is a outer corner
    """
    global regions_, last_kinds_of_wall, last_outer_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index, loop_index_outer_corner
    regions = regions_
    bool_outer_corner = 0
    if (regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['left'] == inf and regions['fleft'] == inf) or ( regions['fleft'] == inf and regions['front'] == inf and regions['left'] == inf and regions['right'] == inf and regions['fright'] == inf):
        bool_outer_corner = 1 # It is a corner
        last_kinds_of_wall[index]='C'
        elapsed_time = time.time() - last_outer_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('C') == len(last_kinds_of_wall) and elapsed_time >= 30:
            last_outer_corner_detection_time = time.time()
            loop_index_outer_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('C')
            print('It is a outer corner')
    return bool_outer_corner

def is_inner_corner():
    """
    Assessment of inner corner in the wall. 
    If the three front regions are inferior than the wall_dist.
    If all the elements in last_kinds_of_wall are 'I' and the last time a real corner was detected is superior or equal to 20 seconds:
        To state_outer_inner a 'I' is appended and 
        The time is restart.
    Returns:
            bool_inner_corner: 0 if it is not a inner corner; 1 if it is a inner corner
    """
    global regions_, wall_dist, last_kinds_of_wall, last_inner_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index_inner_corner, loop_index
    regions = regions_
    bool_inner_corner = 0
    if regions['fright'] < wall_dist and regions['front'] < wall_dist and regions['fleft'] < wall_dist:
        bool_inner_corner = 1
        last_kinds_of_wall[index]='I'
        elapsed_time = time.time() - last_inner_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('I') == len(last_kinds_of_wall) and elapsed_time >= 20:
            last_inner_corner_detection_time = time.time()
            loop_index_inner_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('I')
            print('It is a inner corner')
    return bool_inner_corner






#############################################


def angular(theta):
    global yaw_,is_junction_flag
    integral=0
    error= math.radians(theta)-yaw_
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #rate = rospy.Rate(40) 

    msg1 = Twist()
    #sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom) 
	#positive speed_z value represents clockwise angular velocity of the bot and positive speed_x value represents forward linear velocity of the robot
       
    if -math.pi<yaw_<-0.75*math.pi:
        yaw_= yaw_+2*math.pi
        print("added")
        
    prev_error= error
    error=math.radians(theta)-yaw_
    # print("error is", error)
    # print("yaw is ", yaw_)
    kp= 0.5
    #kd= 0.2
    #ki= 0.5
    delta_t= 0.025
    #integral = integral + error*delta_t
    speed_z = kp*error 
    #kd*(error-prev_error)/delta_t + ki*integral
    if abs(speed_z)<0.03:
        is_junction_flag=0
        speed_z=0
    # print("speed ", speed_z)
    speed_x = 0
    msg1.linear.x = speed_x
    msg1.angular.z = speed_z
    pub.publish(msg1)
    #rate.sleep() 

x_dist=0
y_dist=0
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








################################################################
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











def isValid (y,x):
    #Checks if the given coordinates are valid ones.
        if(y>=0 and y<16 and x<16 and x>=0):
            return True 
        return False


def isEnd(x,y): 
    #Checks if the bot has reached the destination. Implement Distance Functionality.
    if((x,y)==(3,2) or (x,y)==(3,3) or (x,y)==(2,2) or (x,y)==(2,3)):
            return True
    else:
            return False 
    #Need to add the functionality for storing the path in memory.
    #If this returns true then we have to start the fast run.

    

def moveAndUpdate(direction):
    global x 
    global y
    global orient 



    if (direction=='L'):

        orient-=1 

    elif (direction=='R'):
        orient+=1

    elif (direction=='B'):
            orient+=2

    orient%=4

    # xprev=x
    # yprev=y

    # if (orient==0):
    #     y+=1
    # if (orient==1):
    #     x+=1
    # if (orient==2):
    #     y-=1
    # if (orient==3):
    #     x-=1
    x,y=conversion()
    
    
    # visited[15-y][x]+=1

    # if(visited[5-y][x]==2):
    #     x=xprev
    #     y=yprev
    #     orient-=2
    #     orient%=4
    
    

    print("X Coordinate ",x)
    print("Y Coordinate ",y)

def wallfront():
    global lidar_range,G
    wall_dist =abs(min(lidar_range[170:190]))
    # print("front wall distance ",wall_dist)
    print("dead end condition is ",0.15*G)
    return (wall_dist<0.15*G)


def isJunction(x,y,orient):
    global lidar_range
    #Returns true if following node is a junction.

    L = wallLeft()
    R = wallRight()
    F = wallFront()
    f=wallfront() 

    #1 Represents wall is presence.
    print("L+R+F is ",L+R+F)
    print('values of L,R,F  ',L,R,F)
    if(L+R+F ==1):
        print("this is junction")
        moveJunction(x,y,orient,L,R,F)
    elif(L+R+f==3):
        print("this is dead end")
        visited[15-y][x]=2
        orient-=2
        orient%=4
    else:
        # print("used left wall follower")
        left_wall_follower()
        update_index()



def moveJunction(x,y,orient,L,R,F):
    global is_junction_flag,theta
    #Decides upon hierarchy of surrounding cells where to move.
    # L=int(input("Wall Present on Left"))
    # R=int(input("Wall Present on Right"))
    # F=int(input("Wall Present in Front ")) 

    # visited[15-y][x]=0

    minVals =[-1,-1,-1] #Left, Right, Front values.
    minVal =1000 #Stores the cell cost.

    turn = 2 # Stores direction for turning.
    print(turn)

    if(orient==0):
        
        if(not F):
            # minVals[2]=visited[15-(y+1)][x]
            minVals[2]=visited[15-(y+1)][x]
        if(not R):
            minVals[1]=visited[15-y][x+1]
        if(not L):
            minVals[0]=visited[15-y][x-1]
    
    if(orient==1):
        
        if(not F):
            minVals[2]=visited[15-y][x+1]
        if(not R):
            minVals[1]=visited[15-(y-1)][x]
        if(not L):
            minVals[0]=visited[15-(y+1)][x]
    
    if(orient==2):
        
        if(not F):
            minVals[2]=visited[15-(y-1)][x]
        if(not R):
            minVals[1]=visited[15-y][x-1]
        if(not L):
            minVals[0]=visited[15-y][x+1]
    
    if(orient==3):
        
        if(not F):
            minVals[2]=visited[15-y][x-1]
        if(not R):
            minVals[1]=visited[15-(y+1)][x]
        if(not L):
            minVals[0]=visited[15-(y-1)][x]

    for i in range(len(minVals)):
        if minVals[i]!=-1:
            if minVal<minVals[i]:
                minVal=minVals[i]
                turn =i
    
    #Returns direction to turn according to turn.



    if(turn==0):
        moveAndUpdate('L')
        is_junction_flag=1
        theta=90
        print("turn left")
    elif(turn==1):
        moveAndUpdate('R')
        is_junction_flag=1
        theta=-90
        print("turn right")
    elif(turn==2):
        moveAndUpdate('F')
        is_junction_flag=1
        theta=0
        print("move forward")
    else:
        moveAndUpdate('B')
        is_junction_flag=1
        theta=180
        print("turn back")
    

    #Define a Function for turning orientation. 
    #Take care about cost of the junction cell and overall cost.


def left_wall_follower():
    global active_, hz, loop_index,state_,state_
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    loop_index = loop_index + 1
    msg = Twist()

    # State Dispatcher
    if state_ == 0:
        print("enter random wandering")
        msg=random_wandering()
    elif state_ == 1:
        print("enter wall following")
        msg=following_wall()
    elif state_ == 2:
        print("enter rotating")
        msg=rotate()
    else:
        print('Unknown state!')
    
    pub_.publish(msg)

def wallLeft():
    global lidar_range,G
    wall_dist =abs(min(lidar_range[340:360]))
    # print("left wall distance ",wall_dist)
    return (wall_dist<1.3*G)

def wallFront():
    global lidar_range,G
    wall_dist =abs(min(lidar_range[170:190]))
    print("front wall distance ",wall_dist)
    print("condition is ",wall_dist-0.7*G)
    return (wall_dist<1*G)

def wallRight():
    global lidar_range,G
    wall_dist =abs(min(lidar_range[0:20]))
    # print("right wall distance ",wall_dist)
    return (wall_dist<1.3*G)

########################################
prev_index_x,prev_index_y=0,0
def update_index():
    global prev_index_x,prev_index_y,visited
    index_x,index_y=conversion()
    if(abs(index_x-prev_index_x)>=1 or abs(index_y-prev_index_y)>=1):   
        visited[15-index_y][index_x]+=1
    prev_index_x=index_x
    prev_index_y=index_y
    if(visited[15-index_y][index_x]>2):
        visited[15-index_y][index_x]=2










def main():

    global x
    global y 
    global visited
    global orient 
    global historyCount 
    global turn_stack
    global angular_pid_flag,is_junction_flag
    global theta
    # global history 
    #global walkCount


    #######################################
    global pub_, active_, hz, loop_index
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    rate = rospy.Rate(40)
    ################################





    
   

    
    # maze=np.full((16,16),0) #Used for storing wall configuration.

    visited = np.full((16,16),0) #Used for storing the flood array and the costs which shall be used for traversal.
    #maze = np.full((6,6),0)
        
    # x=15  #Stores the location of x coordinate currently robot is at.
    # y=0 #Stores the location of y coordinate currently robot is at.
    x,y=conversion()

    orient = 0 #(orient_inital) #Stores orientation for the robot. 0 for north,1 for east, 2 for south and 3 for west.
    visited[15-0][0]=1

    print(visited)

    while not rospy.is_shutdown():
        # print("congo")
        # sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
        x,y=conversion()
        if not isEnd(x,y):
            if(y>=0 and y<16 and x<16 and x>=0):
                if(is_junction_flag==0): 
                    print(visited)
                    isJunction(x,y,orient)
                else:
                    print("enter rotating of theta")
                    angular(theta)
                
                



        rate.sleep()



            #Execute LWF(x,y,orient) and Update Coordinates Accordingly and also the visited Matrix.    



   


     
visited =[]#Used for storing the flood array and the costs which shall be used for traversal.       
# x=0#Stores the location of x coordinate currently robot is at.
# y=0 #Stores the location of y coordinate currently robot is at.
x,y=conversion()
orient=0#(orient_inital) #Stores orientation for the robot. 0 for north,1 for east, 2 for south and 3 for west.
turn_stack =[] # To be used as a vector for storing shortest paths along with information of turns and distance travelled.
angular_pid_flag,is_junction_flag=0,0

historyCount =0 #Stores the number of steps moved by the bot while reaching the destination.
# history ={} #Dictionary that will store the co-ordinates and orientation of bot 
#     #with co-ordinates as key and orientation as value.
walkCount =0 #TO store the number of cells to be moved while coming back.
theta=0
G = 0.170125

main()
