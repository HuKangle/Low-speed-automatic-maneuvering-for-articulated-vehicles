import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from articulatedVehicle import ArticulatedVehicle
from probabilistic_roadmap import PRM
import numpy as np
import time
from improvedAstar import improved_astar, Node



def start_prm():
    # sx = 1
    # sy = 1
    # gx = 9
    # gy = 4
    # robot_size = 5
    # ox = [1, 1, 1, 4, 4, 8, 8, 8]
    # oy = [3, 4, 5, 1, 2, 4, 5, 6]

    sx = 100.0  # [m]
    sy = 100.0  # [m]
    gx = 500.0  # [m]
    gy = 500.0  # [m]
    robot_size = 5.0  # [m]

    ox = []
    oy = []

    for i in range(600):
        ox.append(i)
        oy.append(0.0)
    for i in range(600):
        ox.append(600.0)
        oy.append(i)
    for i in range(610):
        ox.append(i)
        oy.append(600.0)
    for i in range(610):
        ox.append(0.0)
        oy.append(i)
    for i in range(400):
        ox.append(200.0)
        oy.append(i)
    for i in range(400):
        ox.append(400.0)
        oy.append(600.0 - i)

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "^r")
    plt.plot(gx, gy, "^c")

    plt.grid(True)

    prm = PRM(sx, sy, gx, gy, robot_size, ox, oy)

    rx, ry = prm.find_path()
    reversed_rx = rx[::-1]
    reversed_ry = ry[::-1]

    plt.plot(rx, ry, "-r")
    av = ArticulatedVehicle(plt, sx, sy)
    av.move(1, 0, 0.1)
    av.set_start_position(np.radians(90))

    rx = [100, 100, 100, 100, 100, 100]
    ry = [100, 50, 60, 70, 80, 90, 100]

    av.move_on_path(rx, ry)

    # av.move(300, 0, 0.1)
    # av.move(300, 0, 0.1)
    # av.move(300, 0, 0.1)
    # av.move(300, 0, 0.1)
    # av.move(300, 0, 0.1)
    # av.move(300, 0, 0.1)
    # av.move(300, 0, 0.1)
    # av.move(300, 0, 0.1)
    # av.move(300, -60, 0.1)


    #av.move_on_path(rx, ry)
    #av.move_to_pose(10, 10, 0, 50, 50, 270)

    #print(av.truckHead.get_xy())

    plt.show()


#start_prm()


def inputs(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print('That is not a valid number.')

def move_by_input(av):
    previous_t = time.time()
    now = time.time()
    dt = now - previous_t 
    previous_t = now
    i = 0
    while(i < 180):
        i+=1
        vel = 2
        angle = 0#inputs(">>")
        now = time.time()
        dt = now - previous_t 
        previous_t = now
        av.move(vel,angle,0.1)
        plt.pause(.0001)

def run_astar():
    sx = 100
    sy = 500
    gx = 500
    gy = 100
    img = mpimg.imread('map.png')
    plt.imshow(img)
    plt.plot(sx, sy, "^r")
    plt.plot(gx, gy, "^c")
    goal = Node(gx,gy,0,0)
    start = Node(sx,sy,0,0)
    nodelist = improved_astar(start, goal, img)
    #av = ArticulatedVehicle(plt,start.x,start.y)
    
    nx = list(o.x for o in nodelist)
    ny = list(o.y for o in nodelist)
    
    plt.plot(nx, ny, "-b")
    #move_by_input(av)

    plt.show()

run_astar()