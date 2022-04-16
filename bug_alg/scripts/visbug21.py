#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Twist,Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *

import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
initial_position_ = Point()
initial_position_.x = rospy.get_param('initial_x')
initial_position_.y = rospy.get_param('initial_y')
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
regions_ = None
state_desc_ = ['Go to point', 'wall following', 'done']
state_ = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_odom(msg):
    global position_, yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_
    regions_ = {
        'left':  min(min(msg.ranges[54:89]), 10),
        'fleft': min(min(msg.ranges[8:53]), 10),
        'front':  min(min(min(msg.ranges[0:7]), min(msg.ranges[353:359])) , 10),
        'fright':  min(min(msg.ranges[306:352]), 10),
        'right':   min(min(msg.ranges[270:305]), 10),
    }

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)
        

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
    
def face_the_point():
    global pub, desired_position_
    #point the robot to the destination point
    desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
    err_yaw = desired_yaw - yaw_
    
    while not math.fabs(err_yaw) <= math.pi / 90:
        twist_msg = Twist()
        desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
        err_yaw = desired_yaw - yaw_
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
        pub.publish(twist_msg)
        twist_msg.angular.z = 0
        pub.publish(twist_msg)
    
	

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_, pub
    global srv_client_go_to_point_, srv_client_wall_follower_
    
    rospy.init_node('visbug21')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # set robot position
    model_state = ModelState()
    model_state.model_name = 'turtlebot3_burger'
    
    #model_state.pose.position.x = initial_position_.x
    #model_state.pose.position.y = initial_position_.y
    #resp = srv_client_set_model_state(model_state)
    
    face_the_point()

    #go-to-point
    change_state(0)
    
    print(state_)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        
        if math.sqrt((desired_position_.y - position_.y)**2 + (desired_position_.x - position_.x)**2) < 0.1 :
            twist_msg = Twist()
            twist_msg.angular.z = 0
            twist_msg.linear.x = 0
            pub.publish(twist_msg)
            change_state(2)
            
        
        elif state_ == 0:
            if regions_['front'] > 0.15 and regions_['front'] <= 1 and regions_['front'] <  math.sqrt((desired_position_.y - position_.y)**2 + (desired_position_.x - position_.x)**2):
                change_state(1)
        
        elif state_ == 1:
            desired_yaw = math.atan2(desired_position_.y - position_.y, desired_position_.x - position_.x)
            err_yaw = normalize_angle(desired_yaw - yaw_)
            
            # less than 30 degrees
            if math.fabs(err_yaw) < (math.pi / 6) and \
               regions_['front'] > 1 and (regions_['fright'] < 0.3 or regions_['fleft'] < 0.3):
                print ('less than 30')
                change_state(0)
            
            # between 30 and 90
            if err_yaw > 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['left'] > 0.5 and regions_['fleft'] > 0.5 and regions_['fright'] < 0.5:
                print ('between 30 and 90 - to the left')
                face_the_point()
                change_state(0)
                
            if err_yaw < 0 and \
               math.fabs(err_yaw) > (math.pi / 6) and \
               math.fabs(err_yaw) < (math.pi / 2) and \
               regions_['right'] > 0.5 and regions_['fright'] > 0.5 and regions_['fleft'] < 0.5:
                print ('between 30 and 90 - to the right')
                face_the_point()
                change_state(0)

        rate.sleep()

if __name__ == "__main__":
    main()
