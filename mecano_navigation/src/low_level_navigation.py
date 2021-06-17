#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs import *
from tf2_msgs import *
import tf.transformations
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

import rospy
import time
import actionlib
from move_base_msgs.msg import *

from fuzzy_obstacle_avoidance import fuzy_ranmdom_navigation 

# distances
front_distance = 100
left_distance = 100
right_distance = 100

#position of the robot given by the callback funtion
x_pose = 0
y_pose = 0
theta_angle = 0
theta_goal = 1.5456

#parameters
Kp_rho = 0.1 #9
Kp_alpha = 3.5 #15
Kp_beta = -1 #-3
 
Ki_rho = 0.01
Ki_alpha = 0.00001
Integrator = 0
alpha_integrator = 0

Kd_rho = 1
Kd_alpha = 0.2
Derivator = 0
alpha_derivator = 0 


def moveTest(move_msg):
    x_goal = move_msg.target_pose.pose.position.x
    y_goal = move_msg.target_pose.pose.position.y
    print ("comand to a local",x_goal,y_goal,"goal was received....excecuting")

    x_diff = x_goal - x_pose
    y_diff = y_goal - y_pose
    rho = np.hypot(x_diff, y_diff)
        
    while (rho > 0.25):
        if server.is_preempt_requested():
            result = MoveBaseResult()
            server.set_preempted(result, "Canceled the action of going to the goal")
            return
        
        feedback = MoveBaseFeedback()
        feedback.base_position.pose.position.x = x_pose
        feedback.base_position.pose.position.y = y_pose
        server.publish_feedback(feedback)
        
        x_diff = x_goal - x_pose
        y_diff = y_goal - y_pose
        rho = np.hypot(x_diff, y_diff)
        move_to_pose(x_goal, y_goal, theta_goal, x_pose, y_pose, theta_angle)   


    result = MoveBaseResult()
    server.set_succeeded(result, "Arrived to a local goal!!!!")


def Odom_reader_cb(msg):
   
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, 
    msg.pose.pose.orientation.y, 
    msg.pose.pose.orientation.z, 
    msg.pose.pose.orientation.w])

    tf.transformations.quaternion_from_euler

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    
    global x_pose
    global y_pose
    global theta_angle
    x_pose = x
    y_pose = y
    theta_angle = yaw


def left_distance_cb(msg): 
    global left_distance 
    left_distance = msg.range

def right_distance_cb(msg):
    global right_distance 
    right_distance = msg.range


def front_distance_cb(msg):
    global front_distance
    front_distance = msg.range


def move_to_pose(x_goal, y_goal, theta_goal, x_current,y_current,theta_current):

    x =  x_current
    y = y_current
    theta = theta_current

    x_diff = x_goal - x
    y_diff = y_goal - y

    rho = np.hypot(x_diff, y_diff)

    if rho > 0.25:

        #alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
        #beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        out = 0

        if (left_distance < 0.25 or right_distance < 0.25 or front_distance < 0.25):
            fuzzyAvoider = fuzy_ranmdom_navigation(left_distance, right_distance, front_distance)
            out = fuzzyAvoider.compute_fuzzy_out(left_distance, right_distance, front_distance)
            #alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
            alpha = 0
            alpha = alpha - (out * 0.0314)

            
        else:
            alpha = (np.arctan2(y_diff, x_diff) - theta + np.pi) % (2 * np.pi) - np.pi
            #print "going to goal"
        
        #proportional
        v = Kp_rho * rho
        w = Kp_alpha * alpha #+ Kp_beta * beta  #this is afuture improve: this will help to crontrol the theta goal as well
        
        #integral
        global Integrator
        global alpha_integrator
        alpha_integrator = alpha_integrator + alpha
        Integrator = Integrator + rho
        I_value = Integrator * Ki_rho
        alpha_I_value = alpha_integrator * Ki_alpha
            
        v = v + I_value
        #w = w + alpha_I_value

        #derivative
        global Derivator
        global alpha_derivator
        D_value = Kd_rho * (rho - Derivator)
        alpha_D_value = Kd_alpha * (alpha - alpha_derivator)
        Derivator = rho
        alpha_derivator = alpha
        
        v = v + D_value
        #w = w + alpha_D_value

        if v > 0.5:
            v = 0.5
        if w > 1.9:
            w = 1.9
        if w < -1.9:
            w = -1.9

        if (out < 0):
            v = v * (1.0 - (-out / 100.0))

        if (out >= 0):
            v = v * (1.0 - (out / 100.0))

        #if alpha > np.pi / 2 or alpha < -np.pi / 2:
            #v = -v

        v = 0.3
        alpha_pub.publish(alpha)
        rho_pub.publish(rho)

        t = Twist()
        t.angular.z = w
        t.linear.x = v
        twist_pub.publish(t)

    else:
        t=Twist()
        t.angular.z = 0
        t.linear.x = 0
        twist_pub.publish(t)

if __name__ == '__main__':
    
    rospy.init_node('low_level_navigation')
    
    server = actionlib.SimpleActionServer('local_goal', MoveBaseAction, moveTest, False)
    server.start()

    rospy.Subscriber('odom', Odometry, Odom_reader_cb)

    rospy.Subscriber('ultrasound_right', Range, right_distance_cb)
    rospy.Subscriber('ultrasound_left', Range, left_distance_cb)
    rospy.Subscriber('ultrasound_front', Range, front_distance_cb)

    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    alpha_pub = rospy.Publisher('alpha', Float32, queue_size=1)
    rho_pub = rospy.Publisher('rho', Float32, queue_size = 1)

    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        
        rate.sleep()