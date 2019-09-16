#!/usr/bin/env python

from __future__ import division
import math
import rospy
from pqp_server.srv import *
import random
import math
import numpy as np
import scipy.spatial
import copy
from math import atan2
import time
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist
from tf.transformations import euler_from_quaternion


NumberOfSamples = 50  
#NumberOfNei = 5  
#NumberOfNei = math.e*(1+1/2)*math.log(N_SAMPLE)
MaxLength = 5  



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



class Sample:
    def __init__(self, x, y, cost, id):
        self.x = x
        self.y = y
        self.cost = cost
        self.id = id

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.id)


class KDTree:
    def __init__(self, data):
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        if len(inp.shape) >= 2: 
	    #print inp.shape 
            index = []
            dist = []
            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)
            #print "index is: " + str(index)
            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        index = self.tree.query_ball_point(inp, r)		#using the built-in KDtree searching around the radius of the sample
        return index

def randomQuaternion():
    s= random.random()
    a1 = math.sqrt(1-s)
    a2 = math.sqrt(s)
    theta1 = 2*math.pi*random.random()
    theta2 = 2*math.pi*random.random()
    w = math.cos(theta2)*a2
    x = math.sin(theta1)*a1
    y = math.cos(theta1)*a1
    z = sin(theta2)*theta2
    return (w,x,y,z)

def randomEulerAngles():
    a = 2*math.pi*random.random()-math.pi
    b = math.acos(1-2*random.random())+(math.pi/2)
    if random.random()< 0.5:
        if (b < math.pi): 
            b = b + math.pi
        else: 
            b = b - math.pi
    d = 2*math.pi*random.random()-math.pi
    return (a, b, d)
  
def eulerToRotate(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
    R2 = [R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2]]
    return R2

def PRMroadmap(sx, sy, gx, gy, ox, oy, rr):

    obkdtree = KDTree(np.vstack((ox, oy)).T)

    sample_x, sample_y = generate_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)

    road_map = roadmap(sample_x, sample_y, rr, obkdtree)

    minx, miny = dijkstra(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y)

    return minx, miny

def distance(x1, x2, y1, y2):
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

def collisioncheck(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.sqrt(dx**2 + dy**2)

    if d >= MaxLength:
        return True

    D = rr
    nstep = int(round(d / D))
    
    idxs, dist = okdtree.search(np.matrix([gx, gy]).T)
    if dist[0] <= rr:
        return True  # collision

    for i in range(nstep):
        idxs, dist = okdtree.search(np.matrix([x, y]).T)
        if dist[0] <= rr:
            return True 
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)


   
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
        yrate = (sy-gy) /10
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


def roadmap(sample_x, sample_y, rr, obkdtree):

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
	    
            if not collisioncheck(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])
            
            #if len(edge_id) >= NumberOfNei:
               # break

        road_map.append(edge_id)


    #print road_map
    return road_map


def dijkstra(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y):

    nstart = Sample(sx, sy, 0.0, -1)
    ngoal = Sample(gx, gy, 0.0, -1)

    openlist, closelist = dict(), dict()
    openlist[len(road_map) - 2] = nstart

    while True:
        if len(openlist) == 0:
            print("Path not found!")
            break

        minCostindex = min(openlist, key=lambda o: openlist[o].cost)
        current = openlist[minCostindex]
        #print (current.x, current.y, current.cost, current.id)


        if minCostindex == (len(road_map) - 1):
            print("reached GOAL!")
           # print "current.id is: "+str(current.id)
            ngoal.id = current.id
            ngoal.cost = current.cost
            break

        del openlist[minCostindex]
        closelist[minCostindex] = current

        for i in range(len(road_map[minCostindex])):
            nindex = road_map[minCostindex][i]
	    

            dx = sample_x[nindex] - current.x
            dy = sample_y[nindex] - current.y
	    #print "x, y is"+ str((sample_x[nindex], sample_y[nindex]))
            d = math.sqrt(dx**2 + dy**2)
            sample = Sample(sample_x[nindex], sample_y[nindex],
                        current.cost + d, minCostindex)
            
            if nindex in closelist:
                continue
            if nindex in openlist:
                if openlist[nindex].cost > sample.cost:
                    openlist[nindex].cost = sample.cost
                    openlist[nindex].id = minCostindex
            else:
                openlist[nindex] = sample

    minx, miny = [ngoal.x], [ngoal.y]
    id = ngoal.id
    #print "id is: "+str(id)
    while id != -1:
        #print "id is: " + str(id)
        n = closelist[id]
        minx.append(n.x)
        miny.append(n.y)
        id = n.id

    return minx, miny


def generate_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)
    #print str((maxx, maxy))
    sample_x, sample_y = [], []

    while len(sample_x) <= NumberOfSamples:
        tempx = random.uniform(-2, 10)
        tempy = random.uniform(0, 10)
        #print (tempx, tempy)
        index, dist = obkdtree.search(np.matrix([tempx, tempy]).T)

        if dist[0] >= rr:
            sample_x.append(tempx)
            sample_y.append(tempy)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


    #return dist

def prm():
    #print(__file__ + " start!!")

    # start and goal position
    sx = 5  # [m]
    sy = 9.0  # [m]
    sz = 0.4
    gx = 6.0  # [m]
    gy = 3.0  # [m]
    gz = 0.4
    sa = 0
    sb = 0
    sd = 0
    ga = 0
    gb = 0
    gd = 90
    robot_size = 0.3  # [m]

    ox = []
    oy = []
    if(pqp_client([sx, sy, sz], eulerToRotate((sa, sb, sd)))):
        print "start is in obstacle!"
        exit()
    if(pqp_client([gx, gy, gz], eulerToRotate((ga, gb, gd)))):
        print "goal is in obstacle!"
        exit()
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

   
    #if gx in ox and gy in oy:
    #    print "goal state is in obstacle"
    #    exit()
    #start= time.time()
    minx, miny = PRMroadmap(sx, sy, gx, gy, ox, oy, robot_size)
   # end =time.time()
   # print "time of execution is"
  #  print end - start

    #print minx
    #print miny
    print " done planning"
    path = []

    for i in range(0, len(minx)):
        path.append((minx[i], miny[i]))
    
        
   # dist = 0
   # for i in range(0, len(path) - 1):
   #     dist = dist + distance(path[i][0], path[i + 1][0], path[i][1], path[i + 1][1])
  #  print "distance is: " + str(dist)
  
    print "path is:" 
    print path
    if(len(path)<=1):
        print "No available path"    
        exit()
    if(len(path)>1):
        print "the result path is: "
        #path.reverse()
        for i in range(0, len(path)):
	    print path[i]

    return (path, sa, sb, sd, sz, ga, gb, gd, gz)

def talker(paths):
    print"start communcating with gazebo"

    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    

    x = ModelState()
    x.model_name = "piano2"
    x.pose = Pose()
    ga = paths[5]
    gb = paths[6]
    gd = paths[7]
    temp = paths[0]
    gz = paths[8]
    sa = paths[1]
    sb = paths[2]
    sd = paths[3]
    sz = paths[4]
    x.pose.orientation.w = 1.0
    x.pose.orientation.z = sd
    x.pose.orientation.x = sa
    x.pose.orientation.y = sb
    x.pose.position.z = sz



    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        x.twist = Twist()
	x.twist.linear.y = 0
	x.twist.angular.y=0
    	
        
	for i in range(0, len(temp)):
       	    x.pose.position.x = temp[i][0]
	    x.pose.position.y = temp[i][1]
	    
            if i == len(temp)-1:
                x.pose.position.z = gz
                x.pose.orientation.y = ga*math.pi / 180
	        x.pose.orientation.x = gb*math.pi / 180
                x.pose.orientation.z = gd*math.pi / 180
                
                #print x.pose.orientation.x
                #print x.pose.orientation.y
                #print x.pose.orientation.z
                rospy.loginfo(x)
                pub.publish(x)

	        time.sleep(0.5)
                exit()
            while True:
	    	rantemp = randomEulerAngles()
                ranz = random.uniform(0.7, 2.7)
                x.pose.position.z = ranz
                x.pose.orientation.y = rantemp[1]
	    	x.pose.orientation.x = rantemp[0]
            	x.pose.orientation.z = rantemp[2]
            	if not pqp_client([x.pose.position.x, x.pose.position.y, x.pose.position.z],eulerToRotate(rantemp)):
               	    break
	    #angle_to_turn = atan2 (temp[i+1][1]-temp[i][1], temp[i+1][0]-temp[i+1][0])*math.pi/180
	
                
            	

	    rospy.loginfo(x)
            pub.publish(x)

	    time.sleep(1)
	   # print "one loop done"
	    
	exit()
    
if __name__ == '__main__':
    try:
        #print "starting"
       
        talker(prm())
    except rospy.ROSInterruptException:
        pass
