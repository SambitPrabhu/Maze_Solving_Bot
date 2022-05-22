import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

def main():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  
    rospy.init_node('cmd_robot', anonymous=True)
    rate = rospy.Rate(50) # 40hz

    while not rospy.is_shutdown():
        msg1 = Twist()
	#positive speed_z value represents clockwise angular velocity of the bot and positive speed_x value represents forward linear velocity of the robot
        speed_z = 0
	speed_x = 2
        msg1.linear.x = speed_x
        msg1.angular.z = speed_z
        pub.publish(msg1)
        rate.sleep()
