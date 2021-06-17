#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import math
import time

ox = []
oy = []
show_animation = True

def map_cb(msg):
    max_j = int (msg.info.width )
    max_i = int (msg.info.height)
    print "hello topic"

     # start and goal position
    sx = -1.5  # [m]
    sy = -1.0  # [m]
    gx = 1.5  # [m]
    gy = 1.5  # [m]
    grid_size =  0.1 #[m]
    robot_radius = 0.2 # [m]

    global ox 
    global oy
    actual_position=0
    #detect where the obstacles are in the map
    for i in range (max_i):
        for j in range (max_j):                      
            if msg.data[actual_position]>=50:
                ox.append(j*msg.info.resolution + msg.info.origin.position.x)
                oy.append(i*msg.info.resolution + msg.info.origin.position.y)
            actual_position = actual_position + 1       
    plt.plot(ox, oy, ".k")

    time_before = time.clock()
    print time_before

    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)

    time_after = time.clock() - time_before
    print ("time to process %f: " %time_after) 
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.plot(rx, ry, "-r")
    plt.xlabel('X_Position(m)')
    plt.ylabel('Y_Position(m)')
    plt.show()

class Dijkstra:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.pind = pind # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)

        openset, closedset = dict(), dict()
        openset[self.calc_index(nstart)] = nstart

        while 1:
            c_id = min(openset, key=lambda o: openset[o].cost)
            current = openset[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current.x, self.minx),
                         self.calc_position(current.y, self.miny), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closedset.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                print("Find goal")
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del openset[c_id]

            # Add it to the closed set
            closedset[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in closedset:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in openset:
                    openset[n_id] = node  # Discover a new node
                else:
                    if openset[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        openset[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closedset)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_position(ngoal.x, self.minx)], [
            self.calc_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_position(n.x, self.minx))
            ry.append(self.calc_position(n.y, self.miny))
            pind = n.pind
        return rx, ry

    def calc_position(self, index, minp):
        pos = index*self.reso+minp
        return pos

    def calc_xyindex(self, position, minp):
        return round((position - minp)/self.reso)

    def calc_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.minx)
        py = self.calc_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = int(round((self.maxx - self.minx)/self.reso))
        self.ywidth = int(round((self.maxy - self.miny)/self.reso))
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)
        
        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)] for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break
                    

    def get_motion_model(self):
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

if __name__ == '__main__':
    rospy.init_node('map_reader')
    map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1)
    rospy.Subscriber('map', OccupancyGrid, map_cb)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():        
        rate.sleep()