#!/usr/bin/env python
"""Import statements"""
from math import atan2, sin
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

#initializing the global variables
pose = [0, 0, 0]
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}
def odom_callback(data):
    """Definition of the odometry_callback function."""
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pos_ = data.pose.pose.position
    pose = [pos_.x, pos_.y, euler_from_quaternion([x, y, z, w])[2]]
    #pose contains the ebot's information in the form of (x,y,theta)
def laser_callback(msg):
    """Definition of the laser_callback function."""
    global regions_
    range_max = 10
    #regions_ is the global variable
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), range_max),
        'fright': min(min(msg.ranges[144:287]), range_max),
        'front':  min(min(msg.ranges[288:431]), range_max),
        'fleft':  min(min(msg.ranges[432:575]), range_max),
        'left':   min(min(msg.ranges[576:719]), range_max),
    }

def Waypoints(t):
    '''setting the waypoints to traverse the path.'''
    global pose
    if pose[0] < 6.12:
        x = pose[0] + 0.1
        y = 2*sin(x)*sin(x/2)
    else:
        x = 12.48
        y = 0
    return [x, y]

def control_loop():
    """Python program to move the ebot from the origin to the  goal following the specified path
       and avoiding the concave obstacle"""
    #  Initializes the ROS node 'ebot_controller for the process.
    rospy.init_node('ebot_controller')
    #  To Create a handle to publish messages to the topic /cmd_vel.
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #  To Create a handle to subscribe messages to the topics /ebot/laser/scan and /odom.
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    #  Set the Loop Rate
    rate = rospy.Rate(10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)
    t = 0 #Waypoint index
    while not rospy.is_shutdown():
        global pose, regions_
        ebot_theta = pose[2]
        next_point = Point()
        [next_point.x, next_point.y] = Waypoints(t)
        theta_goal = atan2(next_point.y - pose[1], next_point.x - pose[0])
        steering_angle = theta_goal - ebot_theta
        kp = 1.95 #proportional constant
        if pose[0] <= 6.12 or pose[0] >= 10.25  and pose[0] < 12.2:
            #condition to traverse the curved path
            velocity_msg.linear.x = 0.5
            velocity_msg.angular.z = kp * steering_angle
            t = t + 1
    	else:
            d = 1.5
            if pose[0] >= 12.45:
                #condition to stop the ebot at the goal point
                velocity_msg.linear.x = 0
                velocity_msg.linear.z = 0
            elif pose[0] >= 6.12 and pose[0] <= 10.25:
                #obstacle avoidance
                if regions_['front'] > d and regions_['fleft'] > d and regions_['fright'] > d:
                    velocity_msg.linear.x = 0.2
                    velocity_msg.angular.z = -0.5
                elif regions_['front'] < d and regions_['fleft'] > d and regions_['fright'] > d:
                    velocity_msg.linear.x = 0
                    velocity_msg.angular.z = 0.5
                elif regions_['front'] > d and regions_['fleft'] > d and regions_['fright'] < d:
                    velocity_msg.linear.x = 0.2
                    velocity_msg.angular.z = 0
                elif regions_['front'] > d and regions_['fleft'] < d and regions_['fright'] > d:
                    velocity_msg.linear.x = 0.2
                    velocity_msg.angular.z = -0.5
                elif regions_['front'] < d and regions_['fleft'] > d and regions_['fright'] < d:
                    velocity_msg.linear.x = 0
                    velocity_msg.angular.z = 0.5
                elif regions_['front'] < d and regions_['fleft'] < d and regions_['fright'] > d:
                    velocity_msg.linear.x = 0
                    velocity_msg.angular.z = 0.5
                elif regions_['front'] < d and regions_['fleft'] < d and regions_['fright'] < d:
                    velocity_msg.linear.x = 0
                    velocity_msg.angular.z = 0.5
                elif regions_['front'] > d and regions_['fleft'] < d and regions_['fright'] < d:
                    velocity_msg.linear.x = 0.2
                    velocity_msg.angular.z = -0.5
                else:
                    rospy.loginfo(regions_)



        pub.publish(velocity_msg)

    	print("Controller message pushed at {}".format(rospy.get_time()))
        print("Waypoint index = ", t)
    	rate.sleep()
# Python Main
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass

