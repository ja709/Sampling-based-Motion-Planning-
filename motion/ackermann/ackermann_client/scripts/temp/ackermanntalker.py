#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose,Twist

"""
"""
import matplotlib as mpl
mpl.use('TkAgg')
import matplotlib.pyplot as plt
import random
import math
import copy
import numpy as np
show_animation = False


class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=1.0, goalSampleRate=0, maxIter=0):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand)*10, random.uniform(
                    self.minrand, self.maxrand)*10]
                #print "if: " +str(rnd)
            else:

                rnd = [self.end.x, self.end.y]
                #print "else: "+ str(rnd)

            print "Random node is: " + str((rnd[0], rnd[1]))
            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            #print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            print "nearest node is: " + str((nearestNode.x, nearestNode.y))
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
            print(rnd[1]-nearestNode.y, rnd[0]-nearestNode.x)
            print(theta)

            #print "nearestNode is: "+str((nearestNode.x, nearestNode.y))
            #print (nearestNode.x, nearestNode.y)
            newNode = copy.deepcopy(nearestNode)
            print(newNode.x, newNode.y)
            newNode.x += self.expandDis * math.cos(theta)

            newNode.y += self.expandDis * math.sin(theta)
            print "new node is: "+ str((newNode.x, newNode.y))

            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obstacleList):
                print "the new node is in collision"
                continue

            self.nodeList.append(newNode)
            #print("nNodelist:", len(self.nodeList))

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal is found!")
                break

            if animation:
                self.DrawGraph(rnd)


        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        for (ox, oy) in self.obstacleList:
            plt.plot(ox, oy, "ok", ms=5)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-9, 10, -9, 10])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        #for i in range(0, len(nodeList)):
           # print (nodeList[i].x, nodeList[i].y)
        #print ""
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        #print "dlist is: "+ str(dlist)
        #print(rnd)
        #print (node.x, node.y)
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= 1:
                return False  # collision

        return True  # safe


class Node():
    """
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(x1, x2, y1, y2):
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

def ackermann():
    print("start simple RRT path planning")

    # ====Search Path with RRT====
    obstacleList = []

    for i in np.arange(-7.5, 6.5, 0.5):
        obstacleList.append((-9, i))
        obstacleList.append((10, i))
    for i in np.arange(-9, 10, 0.5):
        obstacleList.append((i, -7.5))
        obstacleList.append((i, 6.5))

    obstacleList.append((-4.5, 1))
    obstacleList.append((-4.2, 1))
    obstacleList.append((-4.5, -7.5))
    obstacleList.append((-4.2, -7.5))
    obstacleList.append((-4.5, -7.5))
    obstacleList.append((-4.2, -7.5))
    for i in np.arange(-7.5, 1, 0.5):
        obstacleList.append((-4.5, i))
        obstacleList.append((-4.2, i))
    obstacleList.append((1.2, 6.5))
    obstacleList.append((1.5, -1.5))
    obstacleList.append((1.2, -1.5))
    obstacleList.append((1.5, 6.5))
    for i in np.arange(-1.5, 6.5, 0.5):
        obstacleList.append((1.2, i))
        obstacleList.append((1.5, i))

    obstacleList.append((6, 2.9))
    obstacleList.append((6.3, 2.9))
    obstacleList.append((6, -4.2))
    obstacleList.append((6.3, -4.2))
    for i in np.arange(-4.2, 2.9, 0.5):
        obstacleList.append((6, i))
        obstacleList.append((6.3, i))

    #test box
    # for i in np.arange(-10, 10, 0.5):
    #     obstacleList.append((i, 10))
    #     obstacleList.append((i, -8))
    # for i in np.arange(-8, 10, 0.5):
    #     obstacleList.append((-8, i))
    #     obstacleList.append((8, i))
    print obstacleList
    start = [-7.5, -6]
    goal = [9, 5]
    # [x,y,size]
    # Set Initial parameters
    rrt = RRT(start=start, goal=goal,
              randArea=[-9, 10], obstacleList=obstacleList)
    path = rrt.Planning(animation=show_animation)
    path.reverse()

    print "the path is: "
    for i in range(0, len(path)):
        print path[i]
    # Draw final path
    dist = 0
    for i in range(0, len(path) - 1):
        dist = dist + distance(path[i][0], path[i + 1][0], path[i][1], path[i + 1][1])
    print "distance is: " + str(dist)
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.show()
	
    return path



def talker(path):
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rate = rospy.Rate(200) # 10hz
    temp = path
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        x = ModelState()
	x.model_name = "ackermann_vehicle"
	x.pose = Pose()
	for i in range(0, len(temp)):
 	    print "(x, y) is: "+ str((temp[i][0], temp[i][1]))
       	    x.pose.position.x = temp[i][0]
	    x.pose.position.y = temp[i][1]
	    x.pose.orientation.w = 1.0
	    x.twist = Twist()



	    rospy.loginfo(x)
            pub.publish(x)
            rate.sleep()
	    print "one loop done"


if __name__ == '__main__':
    try:
        talker(ackermann())
    except rospy.ROSInterruptException:
        pass
