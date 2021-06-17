#!/usr/bin/env python
import rospy
import time
import actionlib
from move_base_msgs.msg import *
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import math
from A_star_class import AStarPlanner
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
y_pose = 0
x_pose = 0

x_traj = []
y_traj = []

def Odom_reader_cb(msg):

    global x_pose
    global y_pose
    
    x_pose = msg.pose.pose.position.x
    y_pose = msg.pose.pose.position.y

def goal_cb(msg):

    x_goal = msg.point.x
    y_goal = msg.point.y

    map_client = rospy.ServiceProxy('static_map', GetMap)
    map_occupancy_grid = map_client()
    path_planner(map_occupancy_grid, x_goal, y_goal)

def path_planner(msg, x_goal, y_goal):
    
    show_animation = True
    max_j = int (msg.map.info.width )
    max_i = int (msg.map.info.height)
 
    # start and goal position
    sx = x_pose  # [m]
    sy = y_pose  # [m]

    gx = x_goal  # [m]
    gy = y_goal  # [m]

    grid_size =  0.1 #[m]
    robot_radius = 0.2 # [m]

    ox = []
    oy = []

    actual_position=0
    #detect where the obstacles are in the map
    for i in range (max_i): 
        for j in range (max_j):                      
            if msg.map.data[actual_position]>=50:
                ox.append(j*msg.map.info.resolution + msg.map.info.origin.position.x)
                oy.append(i*msg.map.info.resolution + msg.map.info.origin.position.y)
            actual_position = actual_position + 1       
    plt.plot(ox, oy, ".k")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, show_animation)
    rx, ry = a_star.planning(sx, sy, gx, gy) 
    
    if show_animation:
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.plot(rx, ry, "-r")
        plt.xlabel('X_Position(m)')
        plt.ylabel('Y_Position(m)')
        plt.show()
    
    global x_traj
    global y_traj

    index = 0
    while index < len(rx):
        x_traj.append(rx[index])
        y_traj.append(ry[index])
        index += 1
        if index >= len(rx):
            x_traj.append(rx[len(rx)-1])
            y_traj.append(ry[len(ry)-1])
            break

    print x_traj
    print y_traj

       
def feedback_cb(feedback):
    print('[Feedback] x: %f'%(feedback.base_position.pose.position.x))
    print('[Feedback] y: %f'%(feedback.base_position.pose.position.y))


if __name__ == '__main__':
    
    rospy.init_node('high_level_planner')

    rospy.Subscriber('odom', Odometry, Odom_reader_cb)
    
    client = actionlib.SimpleActionClient('local_goal', MoveBaseAction)
    client.wait_for_server()
    rospy.wait_for_service('static_map')
    
    rospy.Subscriber('clicked_point', PointStamped, goal_cb)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():     
        if (len(x_traj)>0):
            
            goal = MoveBaseGoal()
                
            goal.target_pose.pose.position.x = x_traj.pop()
            goal.target_pose.pose.position.y = y_traj.pop()

            client.send_goal(goal, feedback_cb=feedback_cb)
            client.wait_for_result()
            print('[Result] State: %d'%(client.get_state()))
            print('[Result] Status: %s'%(client.get_goal_status_text()))

        rate.sleep()

