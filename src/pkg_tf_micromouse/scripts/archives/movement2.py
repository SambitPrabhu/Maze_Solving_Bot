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


##########################
############################## wall follower variables
loop_index = 0              # Number of sampling cycles
loop_index_outer_corner = 0 # Loop index when the outer corner is detected
loop_index_inner_corner = 0 # Loop index when the inner corner is detected
inf = 0.09                    # Limit to Laser sensor range in meters, all distances above this value are 
                            #      considered out of sensor range
wall_dist = 0.075            # Distance desired from the wall
max_speed = 0.1             # Maximum speed of the robot on meters/seconds
p = 15                      # Proportional constant for controller  
d = 0                       # Derivative constant for controller 
angle = 1                   # Proportional constant for angle controller (just simple P controller)
direction = -1              # 1 for wall on the left side of the robot (-1 for the right side)
e = 0                       # Diference between current wall measurements and previous one
angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
dist_front = 0              # Measured front distance
diff_e = 0                  # Difference between current error and previous one
dist_min = 0     

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
    
    #############
    for i in range(360):
        lidar_range[i]=min(msg.ranges[i],0.09)
    
    #############
    
    
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
        change_state(2)
        if(regions['left'] < wall_dist or regions['right'] < wall_dist):
            rotating = 0
    elif regions['fright'] == inf and regions['front'] == inf and regions['right'] == inf and regions['fleft'] == inf and regions['left'] == inf:
        state_description = 'case 0 - random wandering'
        change_state(0)
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
    # msg = Twist()
    # msg.linear.x = max(min( last_vel[0] + random.uniform(-0.01,0.01),0.3),0.1)
    msg_angular_z= max(min( last_vel[1] + random.uniform(-0.1,0.1),1),-1)
    if msg_angular_z == 1 or msg_angular_z == -1:
        msg_angular_z = 0
    # last_vel[0] = msg.linear.x
    last_vel[1] = msg_angular_z
    return msg_angular_z

def following_wall():
    print("enter following_wall")
    """
    PD control for the wall following state. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                    msg.angular.z -> PD controller response
    """
    global wall_dist, max_speed, direction, p, d, angle, dist_min, dist_front, e, diff_e, angle_min
    # msg = Twist()
    # if dist_front < wall_dist:
    #     print("enter 1")
    #     msg.linear.x = 0
    # elif dist_front < wall_dist*2:
    #     print("enter 2")
    #     msg.linear.x = 0.5*max_speed
    # elif abs(angle_min) > 1.75:
    #     print("enter 3")
    #     msg.linear.x = 0.4*max_speed
    # else:
    #     print("enter 4")
    #     msg.linear.x = max_speed
    msg_angular_z = max(min(direction*(p*e+d*diff_e) + angle*(angle_min-((math.pi)/2)*direction), 2.5), -2.5)
    #print 'Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x)
    return msg_angular_z

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
        direction = -direction # Wall in the other side now
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
    # msg = Twist()
    # msg.linear.x = 0
    msg_angular_z = direction*2
    return msg_angular_z

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
    print('coordinates is',x_dist,y_dist)
    # print("angle is ",yaw_)

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

###############################

def isValid (y,x):
    #Checks if the given coordinates are valid ones.
        if(y>=0 and y<16 and x<16 and x>=0):
            return True 
        return False

def flood_initial():  # For generating the initial flooded maze assuming no walls are present.

        global flood

   
        flood[7][7] =0 #Destination cells are given weightage as zero.
        flood[7][8] =0 
        flood[8][7] =0
        flood[8][8] =0


        for i in range(16):
            for j in range(16):
                xdist = min(abs(j-7),np.abs(j-8))
                ydist = min(abs(i-7),np.abs(i-8))
                flood[i][j] = xdist + ydist
def isValid (y,x):
    #Checks if the given coordinates are valid ones.
        if(y>=0 and y<16 and x<16 and x>=0):
            return True 
        return False
        
def isEnd(self): #Checks if the bot has reached the destination.
    if(maze[y][x]==0):
            return True
    else:
            return False 
    #Need to add the functionality for storing the path in memory.
    #If this returns true then we have to start the fast run.

def updateWalls(x,y,orient,L,R,F):

    global maze
    if(L and R and F):
        if (orient==0): 
            maze[15-y][x]= 13
        elif (orient==1): 
            maze[15-y][x]= 12
        elif (orient==2): 
            maze[15-y][x]= 11
        elif (orient==3): 
            maze[15-y][x]= 14

    elif (L and R and not F):
        if (orient==0 or orient== 2): 
            maze[15-y][x]= 9
        elif (orient==1 or orient==3): 
            maze[15-y][x]= 10

    elif (L and F and not R):
        if (orient==0): 
            maze[15-y][x]= 8
        elif (orient==1): 
            maze[15-y][x]= 7
        elif (orient==2): 
            maze[15-y][x]= 6
        elif (orient==3): 
            maze[15-y][x]= 5

    elif (R and F and not L):
        if (orient==0): 
            maze[15-y][x]= 7
        elif (orient==1): 
            maze[15-y][x]= 6
        elif (orient==2): 
            maze[15-y][x]= 5
        elif (orient==3): 
            maze[15-y][x]= 8

    elif(F):
        if (orient==0): 
            maze[15-y][x]= 2
        elif (orient==1): 
            maze[15-y][x]= 3
        elif (orient==2): 
            maze[15-y][x]= 4
        elif (orient==3): 
            maze[15-y][x]= 1

    elif(L):
        if (orient==0): 
            maze[15-y][x]= 1
        elif (orient==1): 
            maze[15-y][x]= 2
        elif (orient==2): 
            maze[15-y][x]= 3
        elif (orient==3): 
            maze[15-y][x]= 4

    elif(R):
        if (orient==0): 
            maze[15-y][x]= 3
        elif (orient==1): 
            maze[15-y][x]= 4
        elif (orient==2): 
            maze[15-y][x]= 1
        elif (orient==3): 
            maze[15-y][x]= 2
    
def isAccessible(x,y,x1,y1):
    #returns True if mouse can move to x1,y1 from x,y (two adjacent cells)

    global maze
    y=15-y
    y1=15-y1
    if(x==x1 and y==y1):
        return False
    
    # if((x==0) and y==5 and y1==4 and x1==0):
    #     return True
    
    # if ((x==5) and y==0 and x1==5 and y1==1):
    #     return True

    # if ((x==5) and y==0 and x1==4 and y1==0):
    #     return True

    if (x==x1):
        if(y<y1):
            if(maze[y][x]==4 or maze[y][x]==5 or maze[y][x]==6 or maze[y][x]==10 or maze[y][x]==11 or maze[y][x]==12 or maze[y][x]==14 ):
                return (False)
            else:
                return(True)
        else:
            if(maze[y][x]==2 or maze[y][x]==7 or maze[y][x]==8 or maze[y][x]==10 or maze[y][x]==12 or maze[y][x]==13 or maze[y][x]==14 ):
                return (False)
            else:
                return(True)
            
    elif (y==y1):
        if(x>x1):
            if(maze[y][x]==1 or maze[y][x]==5 or maze[y][x]==8 or maze[y][x]==9 or maze[y][x]==11 or maze[y][x]==13 or maze[y][x]==14 ):
                return (False)
            else:
                return (True)
        else:
            if(maze[y][x]==3 or maze[y][x]==6 or maze[y][x]==7 or maze[y][x]==9 or maze[y][x]==11 or maze[y][x]==12 or maze[y][x]==13 ):
                return (False)
            else:
                return (True)

def getSurrounds(x,y):
    # Returns x1,y1,x2,y2,x3,y3,x4,y4 the four surrounding square
    y=15-y
    x3= x-1
    y3=y
    x0=x
    y0=y-1
    x1=x+1
    y1=y
    x2=x
    y2=y+1
    if(x1>=16):
        x1=15
    if(y0<0):                        #If overshooting values returning same cell value.
        y0=0
    if(x3<0):
        x3=0
    if(y2>=16):
        y2=15
    return (x0,y0,x1,y1,x2,y2,x3,y3)  #order of cells- north,east,south,west

def isConsistent(x,y):

    global flood
    
    if(flood[15-y][x]==0):
        return True
    #returns True if the value of current square is one greater than the minumum value of an accessible neighbour.

    #One of the neighbouring cells should have value one less
    #than current cell so that mouse can move to it.

    x0,y0,x1,y1,x2,y2,x3,y3 = getSurrounds(x,y)
    val= flood[15-y][x]  #Cost of current cell.
    minVals=[-1,-1,-1,-1]
    if (x0>=0 and y0>=0):
        if (isAccessible(x,y,x0,y0)):
            minVals[0]=flood[y0][x0]
    if (x1>=0 and y1>=0):
        if (isAccessible(x,y,x1,y1)):
            minVals[1]=flood[y1][x1]
    if (x2>=0 and y2>=0):
        if (isAccessible(x,y,x2,y2)):
            minVals[2]=flood[y2][x2]
    if (x3>=0 and y3>=0):
        if (isAccessible(x,y,x3,y3)):
            minVals[3]=flood[y3][x3]

    for i in range(4):
        if minVals[i]== -1:
            pass
        elif minVals[i]== val+1 :
            pass
        elif minVals[i]== val-1 :
           return True
    
    return False

def makeConsistent(x,y):

    x0,y0,x1,y1,x2,y2,x3,y3 = getSurrounds(x,y)

    
    global flood

    if(flood[15-y][x]==0):
        return None


    #Makes the current cell consistent i.e there most be atleast one cell in the surroundings having cost one less than the present cell.

    val= flood[15-y][x]
    minVals=[-1,-1,-1,-1]

    if (x0>=0 and y0>=0):
        if (isAccessible(x,y,x0,y0)):
            minVals[0]=flood[y0][x0]
           
    if (x1>=0 and y1>=0):
        if (isAccessible(x,y,x1,y1)):
            minVals[1]=flood[y1][x1]
       
    if (x2>=0 and y2>=0):
        if (isAccessible(x,y,x2,y2)):
            minVals[2]=flood[y2][x2]
           
    if (x3>=0 and y3>=0):
        if (isAccessible(x,y,x3,y3)):
            minVals[3]=flood[y3][x3]
            

    for i in range(4):
        if minVals[i]== -1: #not accessible.
            minVals[i]= 1000 # Assigning a high cost.

    minVal= min(minVals) #finds the minimum cost of nearest accessible cell.
    # if(y==5 and x==0):
    #     flood[y][x]=flood[4][0]+1
    # else:
    flood[15-y][x]= minVal+1 #Updates the cost of present cell accordingly.

def floodFill(x,y,xprev,yprev):

    global flood 
    #Updates the flood matrix such that each and every cell is consistent. 

    if(flood[15-y][x]==0):
        return None

    if not isConsistent(x,y):
        makeConsistent(x,y)

    #Previous Cell is represented by xprev and yprev
        
    stack=[]
    stack.append(x)
    stack.append(15-y)
    x0,y0,x1,y1,x2,y2,x3,y3= getSurrounds(x,y)
    if(x0>=0 and y0>=0):
        if (isAccessible(x,y,x0,y0)):
            stack.append(x0)
            stack.append(y0)
    if(x1>=0 and y1>=0):
        if (isAccessible(x,y,x1,y1)):
            stack.append(x1)
            stack.append(y1)
    if(x2>=0 and y2>=0):
        if (isAccessible(x,y,x2,y2)):
            stack.append(x2)
            stack.append(y2)
    if(x3>=0 and y3>=0):
        if (isAccessible(x,y,x3,y3)):
            stack.append(x3)
            stack.append(y3)

    while (len(stack)!= 0):
        yrun= stack.pop()
        xrun= stack.pop() #May have to change

        if isConsistent(xrun,yrun):
            pass
        else:
            makeConsistent(xrun,yrun)
            stack.append(xrun)
            stack.append(yrun)
            x0,y0,x1,y1,x2,y2,x3,y3= getSurrounds(xrun,yrun)
            if(x0>=0 and y0>=0):
                if (isAccessible(xrun,yrun,x0,y0)):
                    stack.append(x0)
                    stack.append(y0)
            if(x1>=0 and y1>=0):
                if (isAccessible(xrun,yrun,x1,y1)):
                    stack.append(x1)
                    stack.append(y1)
            if(x2>=0 and y2>=0):
                if (isAccessible(xrun,yrun,x2,y2)):
                    stack.append(x2)
                    stack.append(y2)
            if(x3>=0 and y3>=0):
                if (isAccessible(xrun,yrun,x3,y3)):
                    stack.append(x3)
                    stack.append(y3)

def isChannel(x,y):
    global maze 
    return (maze[15-y][x]==9 or maze[15-y][x]==10) #Checks if the present cell is a channel.

def toMove(x,y,xprev,yprev,orient):
    '''Returns the direction to turn into L,F,R or B
    '''
    global flood

    x0,y0,x1,y1,x2,y2,x3,y3 = getSurrounds(x,y)
    val= flood[15-y][x]
    prev=0 #Stores the cell from which we have come.

    minVals=[1000,1000,1000,1000]

    if (isAccessible(x,y,x0,y0)):
        if (x0==xprev and y0==yprev):
            prev=0
        minVals[0]= flood[y0][x0]

    if (isAccessible(x,y,x1,y1)):
        if (x1==xprev and y1==yprev):
            prev=1
        minVals[1]= flood[y1][x1]

    if (isAccessible(x,y,x2,y2)):
        if (x2==xprev and y2==yprev):
            prev=2
        minVals[2]= flood[y2][x2]

    if (isAccessible(x,y,x3,y3)):
        if (x3==xprev and y3==yprev):
            prev=3
        minVals[3]= flood[y3][x3]

    minVal=minVals[0] #Stores the minVal.
    minCell=0 # Stores the cell which has minimum cost.
    noMovements=0
    for i in minVals:
        if (i!=1000):
            noMovements+=1

    #noMovements=1 implies only one cell is accessible to it which is the previous cell.



    for i in range(4):
        if (minVals[i]<minVal):
            if (noMovements==1): 
                minVal= minVals[i]
                minCell= i
            else:
                if(i==prev):
                    pass
                else:
                    minVal= minVals[i]
                    minCell= i
    
    k = 0 # Counter for storing number of elements having the same minimum cost. 
    k = minVals.count(minVal) 
    #or (k>1 and minVals.count(minVals[orient])>1)


    if(minCell==orient ):
            return ('F')
    elif((minCell==orient-1) or (minCell== orient+3)):
            return('L')
    elif ((minCell==orient+1) or (minCell== orient-3)):
            return('R')
    else:
        return('B')

def orientation(orient,turning):
    if (turning== 'L'):
        orient-=1
        if (orient==-1):
            orient=3
    elif(turning== 'R'):
        orient+=1
        if (orient==4):
            orient=0
    elif(turning== 'B'):
        if (orient==0):
            orient=2
        elif (orient==1):
            orient=3
        elif (orient==2):
            orient=0
        elif (orient==3):
            orient=1

    return(orient)
        
def moveAndUpdate(direction): #orient being updated
    global x 
    global y
    global orient 
    global xprev
    global yprev

    if (direction=='L'):
            orient-=1 

    elif (direction=='R'):
            orient+=1

    elif (direction=='B'):
            orient+=2

    orient%=4

    xprev=x
    yprev=15-y
    #api.updateCoordinates()
    #return orient

    print("X Coordinate ",x)
    print("Y Coordinate ",15-y)
    print(maze)
    print(flood)

def pruneHistory():

    global x
    global y

    findCell = [x,15-y]
    global historyCount
    global history
    
    #findCell represents the element of the dictionary. It is a tuple.
    i = historyCount -2 
    while(i>=0):
        cell = history[i] 
        if(cell[0] == findCell[0] and cell[1]==findCell[1]):
            historyCount = i+1 # May have to change later.
            return None
        i-=1

def addToHistory():
    #Stores the x,y and orientation of the robot at the particular cell.
    global x
    global y 
    global orient
    global historyCount
    global history
    
    history[historyCount] = (x,15-y,orient) 
    print(history)
    historyCount+=1 

    return None

def fastRun(): 

    global flood
    global walkCount
    global historyCount
    global orient

    while True:

        orient = history[walkCount][2] 
        # api.updateCoordinates()
        walkCount+=1 

        if(walkCount==historyCount):
            print("Fast Run Executed. Reached Destination")
            goHome()
            break


    global orient
    global x
    global y

    if(orient==0):
        y-=1
    elif(orient==1):
        x+=1
    elif(orient==2):
        y+=1
    else:
        x-=1
    
    print("X Coordinate is ", x)
    print("Y Coordinate is ",y)
  
def goHome():

   global walkCount
   global orient

   while True:
    walkCount-=1 

    orient = (history[walkCount][2]+2)%4 #Orientation should be just reverse of original run.
    #Now it needs to just move one block ahead.
    # api.updateCoordinates()

    if walkCount==1:
       print("Reached Home")
       fastRun() #Implement Fast Run. Breaks this.
       break


#############################

def clbk_odom(msg):
    print("enter")
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
    
    # print(x_dist, y_dist,"actual coordinates")
    # print("angle is ",yaw_)

def linear_pid(orient):
    global limcoord, G, W, x_dist, y_dist, flag,target_x,target_y,flag2,flag3

    msg=Twist()
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
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
            print('target coordinates :',target_x,target_y)
            flag=1
        error = y_dist - target_y
        msg.linear.x = 0.9*error
        # msg.angular.z = 0
        if abs(msg.linear.x) < 0.01:
            print("enter critical region")
            msg.linear.x = 0
            flag= 0
            flag2=0
            flag3=0
        if(msg.linear.x>0.4):
            msg.linear.x=0.4
        elif(msg.linear.x<-0.4):
            msg.linear.x=-0.4
        # pub.publish(msg)
        return msg.linear.x
        # print("vel: ",msg.linear.x)
        # print("error: ",error)
        
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
        msg.linear.x = 0.9*error
        # msg.angular.z = 0
        if abs(msg.linear.x) < 0.01:
            msg.linear.x = 0
            flag= 0
            flag2=0
            flag3=0
        if(msg.linear.x>0.4):
            msg.linear.x=0.4
        elif(msg.linear.x<-0.4):
            msg.linear.x=-0.4
        # pub.publish(msg)
        return msg.linear.x
        
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
        msg.linear.x = 0.9*error
        # msg.angular.z = 0
        if abs(msg.linear.x) < 0.01:
            msg.linear.x = 0
            flag= 0
            flag2=0
            flag3=0
        if(msg.linear.x>0.4):
            msg.linear.x=0.4
        elif(msg.linear.x<-0.4):
            msg.linear.x=-0.4
        # pub.publish(msg)
        return msg.linear.x
        
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
        msg.linear.x = 0.9*error
        # msg.angular.z = 0
        if abs(msg.linear.x) < 0.01:
            msg.linear.x = 0
            flag= 0
            flag2=0
            flag3=0
        if(msg.linear.x>0.4):
            msg.linear.x=0.4
        elif(msg.linear.x<-0.4):
            msg.linear.x=-0.4
        # pub.publish(msg)
        return msg.linear.x

def angular(theta):
    global yaw_
    global flag2,flag
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
    print("error is", error)
    print("yaw is ", yaw_)
    kp= 0.5
    #kd= 0.2
    #ki= 0.5
    delta_t= 0.025
    #integral = integral + error*delta_t
    speed_z = kp*error 
    #kd*(error-prev_error)/delta_t + ki*integral
    if abs(speed_z)<0.03:
        flag2=1
        flag =1
        speed_z=0
    print("speed ", speed_z)
    speed_x = 0
    msg1.linear.x = speed_x
    msg1.angular.z = speed_z
    pub.publish(msg1)
    #rate.sleep()            

def wallLeft():
    global lidar_range,G
    wall_dist =abs(min(lidar_range[240:360]))
    return (wall_dist<1.3*G)

def wallFront():
    global lidar_range,G
    wall_dist =abs(min(lidar_range[120:240]))
    return (wall_dist<1.3*G)

def wallRight():
    global lidar_range,G
    wall_dist =abs(min(lidar_range[0:120]))
    return (wall_dist<1.3*G)

def main():

    global x
    global y 
    global maze 
    global flood 
    global orient 
    global xprev
    global yprev
    global historyCount 
    global history 
    global walkCount
    global yaw_,x_dist,y_dist,flag,pub_,active_,loop_index
    global flag2,flag3

    xprev=15
    yprev=0  
    maze=np.full((16,16),0) #Used for storing wall configuration.
    maze[0][15]= 11
    flood = np.full((16,16),255) #Used for storing the flood array and the costs which shall be used for traversal.    
    x=15  #Stores the location of x coordinate currently robot is at.
    y=0 #Stores the location of y coordinate currently robot is at.
    orient = 0 #(orient_inital) #Stores orientation for the robot. 0 for north,1 for east, 2 for south and 3 for west.
    flood_initial() 

    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)   
    # msg = Twist()
    
    sub_lidar = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_robot', anonymous=True)
    rate = rospy.Rate(40) # 40hz

    while not rospy.is_shutdown():
        # xprev = x
        # yprev = y
        x,y = conversion()
        # L= wallLeft()
        # R= wallRight()
        # F= wallFront()
        # updateWalls(x,y,orient,L,R,F)
        loop_index = loop_index + 1
        print("coord ",x,y)      
        print("flood is ",flood[y][x])
        if (flood[y][x]!=0):
            if flag == 0:
                if(flag3==0):
                    if(maze[y][x]==0):
                        L= wallLeft()
                        R= wallRight()
                        F= wallFront()
                        updateWalls(x,y,orient,L,R,F)
                        
                    direction = toMove(x,y,xprev,yprev,orient)
                    if(not isChannel(x,y)):
                        if(not (x==0 and y==15)):
                            floodFill(x,y,xprev,yprev)
                    moveAndUpdate(direction)
                    # addToHistory()
                    # pruneHistory() # Add Function for adding to history. Cross check once.
                    flag3=1

                if orient == 0:
                    angular(0)
                    # flag2=1
                    # flag=1
                elif orient == 1:
                    angular(-90)
                elif orient == 2:
                    angular(180)
                elif orient == 3:
                    angular(90)
            
               
            elif(flag2==1):
                msg=Twist()
                msg.linear.x=linear_pid(orient)
                ##########################
                # State Dispatcher
                if state_ == 0:
                    print("enter 0 state")
                    msg.linear.z=random_wandering()
                elif state_ == 1:
                    print("enter 1 state")
                    msg.linear.z=following_wall()
                elif state_ == 2:
                    print("enter 2 state")
                    msg.linear.z=rotate()
                else:
                    print('Unknown state!')
                pub_.publish(msg)

        else:

            print("Eureka!! Path has been found")
            walkCount=historyCount
            print(flood)
            print(historyCount)
            print()
            #Now it will go back.
            goHome()

        rate.sleep()


if __name__ == '__main__':
    maze =[] #Used for storing wall configuration.
    flood =[]#Used for storing the flood array and the costs which shall be used for traversal.       
    x=0#Stores the location of x coordinate currently robot is at.
    y=0 #Stores the location of y coordinate currently robot is at.
    orient=0#(orient_inital) #Stores orientation for the robot. 0 for north,1 for east, 2 for south and 3 for west.
    xprev =0
    yprev =15
    historyCount =1 #Stores the number of steps moved by the bot while reaching the destination.
    history ={} #Dictionary that will store the co-ordinates and orientation of bot 
    history[0]=(0,15,0)
    #with co-ordinates as key and orientation as value.
    walkCount =0 #TO store the number of cells to be moved while coming back.
    
    ###################
    x_dist = 0
    y_dist = 0
    yaw_=0
    target_x = 0
    target_y = 0
    flag=0
    ####################

    flag2=0 # used for angular pid
    flag3=0 #used for skipping stable flood fill code


    main()




