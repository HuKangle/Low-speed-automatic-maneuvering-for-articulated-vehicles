import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
from operator import attrgetter
from articulatedVehicle import ArticulatedVehicle
import time

class Node:
    def __init__(self, x, y,gscore, fscore):
        self.x = x
        self.y = y
        self.gscore = gscore
        self.fscore = fscore

def reconstructPath(cameFrom, current):
    #print("reconstructing from ", current.x,current.y)
    totalPath = []
    while current in cameFrom.keys():
        current = cameFrom[current]
        totalPath.append(current)
    return list(reversed(totalPath))

def a_star(start,goal,obstacles):
    closedNodes, openNodes = dict(), dict()
    openNodes[calc_index(start)] = start
    cameFrom = {}
    motion = get_motion_model()
    while openNodes:
        id = min(openNodes, key=lambda o: openNodes[o].fscore)
        current = openNodes[id]

        if current.x == goal.x and current.y == goal.y:
            return reconstructPath(cameFrom, current)

        del openNodes[id]
        closedNodes[id] = current

        for i in range(len(motion)):
            node = Node(current.x + motion[i][0],
                        current.y + motion[i][1],
                        current.gscore + motion[i][2],
                        0)
            
            n_id = calc_index(node)
            if n_id in closedNodes:
                continue

            if is_collision(node,obstacles):
                continue

            node.fscore = node.gscore + calcHeuristic(goal, node)

            if n_id not in openNodes:
                openNodes[n_id] = node
            else:
                if openNodes[n_id].gscore >= node.gscore:
                    openNodes[n_id] = node

            cameFrom[node] = current

def calc_index(node):
    return node.y * 1152 + node.x

def calcHeuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)

def is_collision(node, graph):
    try:
        for i in range(20):
            pos1 = graph[node.y+i][node.x+i]
            pos2 = graph[node.y-i][node.x-i]
            pos3 = graph[node.y][node.x-i]
            pos4 = graph[node.y][node.x+i]
            pos5 = graph[node.y-i][node.x]
            pos6 = graph[node.y+i][node.x]
            if pos1[0] == 0 or pos2[0] == 0 or pos3[0] == 0 or pos4[0] == 0 or pos5[0] == 0 or pos6[0] == 0:
                return True
        return False
    except IndexError:
        return True

def get_motion_model():
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

def paint(nodeList, c):
    xs = []
    ys = []
    for n in nodeList:
        xs.append(n.x)
        ys.append(n.y)
    plt.plot(xs,ys,c)
        
def improved_astar(start,goal,obs):
    p = a_star(start,goal,obs)
    #paint(p,"-r")
    print(len(p))
    if len(p) > 2:
        for index, node in enumerate(p):
            if index <= len(p)-2:
                i = 2
                len(p)
                while True:
                    if (index+i < len(p)-2):
                        dtwo = p[index+i]
                        if verify_line(node,dtwo,obs):
                            del p[index+i-1]
                        else:
                            break
                        i += 1
                    else:
                        break

    return p

def verify_line(startn,endn,graph):
    if startn.x == endn.x or startn.y == endn.y:
        return True
    M = (startn.y-endn.y)/(startn.x-endn.x)
    B = (startn.x*startn.y - endn.x*startn.y)/(startn.x-endn.x)
    x = startn.x
    if x < endn.x:
        while x < endn.x:
            y = M*x + B
            if y < 0:
                return True
            if is_collision(Node(x,int(y),0,0), graph):
                return False  
            x += 1
    else:
        while x > endn.x:
            y = M*x + B
            if is_collision(Node(x,int(y),0,0), graph):
                return False  
            x -= 1

    return True