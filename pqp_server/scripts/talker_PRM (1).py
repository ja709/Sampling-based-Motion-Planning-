#!/usr/bin/env python

from __future__ import division
import math
import rospy
from pqp_server.srv import *
import random
import copy
import math
import numpy as np
import scipy.spatial
import matplotlib as mpl
from math import atan2
mpl.use('TkAgg')
import time
import matplotlib.pyplot as plt
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist
from tf.transformations import euler_from_quaternion

# parameter
N_SAMPLE = 500  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
#N_KNN = math.e*(1+1/3)*math.log(N_SAMPLE)
#print N_KNN
MAX_EDGE_LEN = 2  # [m] Maximum edge length

show_animation = False

def pqp_client(T, R):
    if len(T) != 3 or len(R) != 9:
        print "Incorrect list size for pqp request"
        return True
    rospy.wait_for_service('pqp_server')
    try:
        pqp_server = rospy.ServiceProxy('pqp_server', pqpRequest)
        result = pqp_server(T, R)
        #print "testinginging"
	#print isinstance(result, bool)
        
	if str(result)==  "result: False":
		return False
	else:
		return True
        
    except rospy.ServiceException, e:
        print "Service Call Failed: %s"%e



class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        u"""
        Search NN

        inp: input data, single frame or multi frame

        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        u"""
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


def PRM_planning(sx, sy, gx, gy, ox, oy, rr):

    obkdtree = KDTree(np.vstack((ox, oy)).T)

    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_roadmap(sample_x, sample_y, rr, obkdtree)

    rx, ry = dijkstra_planning(
        sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)

    return rx, ry

def distance(x1, x2, y1, y2):
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    nstep = int(round(d / D))

    for i in range(nstep):
        idxs, dist = okdtree.search(np.matrix([x, y]).T)
        if dist[0] <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    idxs, dist = okdtree.search(np.matrix([gx, gy]).T)
    if dist[0] <= rr:
        return True  # collision

    T =[gx, gy, 0.7]
    R = [1,0,0,0,1,0,0,0,1]
    if pqp_client(T, R):            #check if the goal node is located in an obstacle
	#print str((gx, gy))
	#print pqp_client(T, R)
        return True


    if(sx < gx and sy < gy):
        xrate = (gx-sx)/10
        yrate = (gy-sy)/10
        tempx = sx+xrate
        tempy = sy+yrate
        while(tempx<gx and tempy<gy):
            T=[tempx, tempy, 1]
            if (pqp_client(T, R)):
		#print "2"
                return True
            tempx = tempx+xrate
            tempy = tempy+yrate

    if (sx < gx and sy > gy):
        xrate = (gx-sx)/10
        yrate = (sy-gy)/10
        tempx = sx+xrate
        tempy = sy-yrate
        while(tempx<gx and tempy>gy):
            T = [tempx, tempy, 1]
            if (pqp_client(T, R)):
		#print "3"
                return True
            tempx = tempx+xrate
            tempy = tempy-yrate
    if (sx == gx and sy < gy):
        yrate = (gy-sy)/10
        tempy = sy+yrate
        while(tempy<gy):
            T=[sx, tempy, 1]
            if (pqp_client(T, R)):
		#print "4"
                return True
            tempy = tempy+yrate
    if (sx == gx and sy > gy):
        yrate = (sy-gy) / 10
        tempy = sy-yrate
        while (tempy > gy):
            T = [sx, tempy, 1]
            if (pqp_client(T, R)):
		#print "5"
                return True
            tempy = tempy - yrate
    if(sx > gx and sy > gy):
        xrate = (sx-gx)/10
        yrate = (sy-gy)/10
        tempx = sx - xrate
        tempy = sy - yrate
        while (tempx>gx and tempy>gy):
            T = [tempx, tempy, 1]
            if (pqp_client(T,R)):
		#print "6"
                return True
            tempx = tempx-xrate
            tempy = tempy-yrate

    if(sx > gx and sy < gy):
        xrate = (sx-gx)/10
        yrate = (gy-sy)/10
        tempx = sx-xrate
        tempy = sy+yrate
        while (tempx >gx and tempy <gy):
            T = [tempx, tempy, 1]
            if (pqp_client(T, R)):
		#print "7"
                return True
            tempx = tempx-xrate
            tempy = tempy+yrate
    if(sx > gx and sy == gy):
        xrate = (sx-gx)/10
        tempx = sx-xrate
        while(tempx>gx):
            T = [tempx, sy, 1]
            if (pqp_client(T,R)):
		#print "8"
                return True
            tempx = tempx-xrate

    if(sx < gx and sy == gy):
        xrate = (gx-sx)/10
        tempx = sx+xrate
        while(tempx<gx):
            T = [tempx, sy, 1]
            if (pqp_client(T,R)):
		#print "9"
                return True
            tempx = tempx+xrate


    return False  # OK


def generate_roadmap(sample_x, sample_y, rr, obkdtree):
    """
    Road map generation

    sample_x: [m] x positions of sampled points
    sample_y: [m] y positions of sampled points
    rr: Robot Radius[m]
    obkdtree: KDTree object of obstacles
    """

    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(nsample), sample_x, sample_y):

        index, dists = skdtree.search(
            np.matrix([ix, iy]).T, k=nsample)
        inds = index[0][0]
        edge_id = []
        #  print(index)
        #print "inds are: "+str(inds)
        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)
    #print road_map
    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    while True:
        if len(openset) == 0:
            print("Cannot find path")
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]
        #print (current.x, current.y, current.cost, current.pind)
        # show graph
        if show_animation and len(closedset.keys()) % 2 == 0:
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
           # print "current.pind is: "+str(current.pind)
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
	    

            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
	    #print "x, y is"+ str((sample_x[n_id], sample_y[n_id]))
            d = math.sqrt(dx**2 + dy**2)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)
            
            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    # generate final course
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    #print "pind is: "+str(pind)
    while pind != -1:
        #print "pind is: " + str(pind)
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):

    for i in range(len(road_map)):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)
    #print str((maxx, maxy))
    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        tx = random.uniform(-10, 10)
        ty = random.uniform(-10, 10)
        #print (tx, ty)
        index, dist = obkdtree.search(np.matrix([tx, ty]).T)

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


    return dist
def prm():
    #print(__file__ + " start!!")

    # start and goal position
    sx = -0.5  # [m]
    sy = 9.0  # [m]
    gx = 6.0  # [m]
    gy = 3.0  # [m]
    robot_size = 0.3  # [m]

    ox = []
    oy = []

    #old version
    # for i in np.arange(2, 10, 0.2):
    #     ox.append(0)
    #     oy.append(i)

    # for i in np.arange(0,5.8, 0.2):
    #     ox.append(i)
    #     oy.append(2)
    # for i in np.arange(5.8, 10, 0.2):
    #     ox.append(i)
    #     oy.append(0)
    # for i in np.arange(10,12, 0.2):
    #      oy.append(i)
    #      ox.append(6)
    # for i in np.arange(0, 6, 0.2):
    #     ox.append(i)
    #     oy.append(10)
    # for i in np.arange(6, 10, 0.2):
    #     oy.append(12)
    #     ox.append(i)
    # for i in np.arange(0,12, 0.2):
    #     ox.append(10)
    #     oy.append(i)
    # for i in np.arange(0,2, 0.2):
    #     ox.append(5.8)
    #     oy.append(i)

    #new version
    for i in np.arange(-10, 10, 0.2):
        ox.append(i)
        oy.append(10)
        ox.append(10)
        oy.append(i)
        ox.append(-10)
        oy.append(i)
        ox.append(i)
        oy.append(0)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        #plt.axis("equal")
    #if gx in ox and gy in oy:
    #    print "goal state is in obstacle"
    #    exit()

    rx, ry = PRM_planning(sx, sy, gx, gy, ox, oy, robot_size)
    #print rx
    #print ry
    print " done planning"
    path = []
    
    for i in range(0, len(rx)):
        path.append((rx[i], ry[i]))
    
        #dist = 0
        #for i in range(0, len(path) - 1):
        #    dist = dist + distance(path[i][0], path[i + 1][0], path[i][1], path[i + 1][1])
        #print "distance is: " + str(dist)
    
    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()
    print "path is:" 
    #print path
    if(len(path)<=1):
        print "No available path"    
        exit()
    if(len(path)>1):
        print "the result path is: "
	path.reverse()
        for i in range(0, len(path)):
	    
	    print path[i]
            
            return path

def talker(path):
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    temp = path
	
    x = ModelState()
    x.model_name = "piano2"
    x.pose = Pose()
	

    x.pose.orientation.y = 0
    x.pose.orientation.w = 1.0
    x.pose.orientation.x = 0
    x.pose.orientation.z = 0

    




    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        x.twist = Twist()
	x.twist.linear.y = 0
	x.twist.angular.y=0
    	
        
	for i in range(0, len(temp)):
       	    x.pose.position.x = temp[i][0]
	    x.pose.position.y = temp[i][1]
	    x.pose.position.z = 0.7
	    #angle_to_turn = atan2 (temp[i+1][1]-temp[i][1], temp[i+1][0]-temp[i+1][0])*math.pi/180
	    x.pose.orientation.z =  random.uniform(0, 1)
  	    rot_q = x.pose.orientation
            (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
            
           
	    

            if(i < len(temp) - 1):
	    	angle_to_goal = atan2 (temp[i+1][1]-temp[i][1], temp[i+1][0]-temp[i+1][0])


	    	if abs(angle_to_goal - theta) > 0.1:
	            x.twist.linear.x = 0.0
		    x.twist.angular.z = 0.0
	        else:
		    x.twist.linear.x = 0.5
		    x.twist.angular.z = 0.0



	    #x.twist = Twist()
	    #x.twist.linear.x = randint(-60, 60)*math.pi/180
	    
	    rospy.loginfo(x)
            pub.publish(x)

	    time.sleep(0.8)
	   # print "one loop done"
	    
	exit()
    
if __name__ == '__main__':
    try:
        talker(prm())
    except rospy.ROSInterruptException:
        pass
