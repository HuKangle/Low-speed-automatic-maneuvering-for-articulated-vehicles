import matplotlib.pyplot as plt
import matplotlib as mpl
import math
import numpy as np
import matplotlib.patches as patches

class ArticulatedVehicle:

    def __init__(self, fig, ax):
        self.fig = fig
        self.ax = ax
        self.velocity = 0
        self.vx = 0
        self.vy = 0
        self.trailerW = 65
        trailerH = 26
        self.headW = 30
        headH = 26
        self.startPointX = 155
        self.startPointY = 213
        self.headAngle = 0
        self.trailerAngle = 0
        self.max_v = 2
        self.max_th = 1
        self.save_hx = [self.startPointX]
        self.save_hy = [self.startPointY]
        self.save_tx = [self.startPointX - (self.headW/2 * math.cos(math.radians(self.headAngle))) - (self.trailerW * math.cos(math.radians(self.trailerAngle)))]
        self.save_ty = [self.startPointY - (self.headW/2 * math.sin(math.radians(self.headAngle))) - (self.trailerW * math.sin(math.radians(self.trailerAngle)))]
        
        self.oldTrailerPose = {
            'x': self.startPointX - (self.headW/2 * math.cos(math.radians(self.headAngle))) - (self.trailerW * math.cos(math.radians(self.trailerAngle))),
            'y': self.startPointY - (self.headW/2 * math.sin(math.radians(self.headAngle))) - (self.trailerW * math.sin(math.radians(self.trailerAngle))),
            'theta': self.trailerAngle
        }



        hW = self.headW / 2
        hH = headH / 2
        tW = self.trailerW 
        tH = trailerH / 2

        p0 = [self.startPointX + hW, self.startPointY - hH]
        p1 = [self.startPointX + hW, self.startPointY + hH]
        p2 = [self.startPointX - hW, self.startPointY + hH]
        p3 = [self.startPointX - hW, self.startPointY - hH]
        p4 = [self.startPointX - hW, self.startPointY - tH]
        p5 = [self.startPointX - hW - tW, self.startPointY - tH]
        p6 = [self.startPointX - hW - tW, self.startPointY + tH]
        p7 = [self.startPointX  - hW, self.startPointY + tH] 

        headVectors = np.array([p0,p1,p2,p3])
        trailerVectors = ([p4,p5,p6,p7])
        self.truckHead = patches.Polygon(headVectors, fill=None)
        self.truckTrailer = patches.Polygon(trailerVectors, fill=None)
        ax.add_patch(self.truckHead)
        ax.add_patch(self.truckTrailer)

    
    def move(self, vel, angle, dt):
        self.__updateHead(vel,angle, dt)

    def __updateHead(self,vel,angle, dt):
        self.velocity += vel

        angleChange = self.velocity/self.headW * math.tan(np.radians(angle))
        self.headAngle += angleChange
        
        theta = np.radians(angleChange)
        

        oldX = self.startPointX
        oldY = self.startPointY

        #####################
        #   X is the startPoint
        #     o------------o
        #     |            |
        #     |      X     |
        #     |            |
        #     o------------o
        self.startPointX += (dt * self.velocity) * math.cos(np.radians(self.headAngle))
        self.startPointY += (dt * self.velocity) * math.sin(np.radians(self.headAngle))
        
        self.save_hx.append(self.startPointX)
        self.save_hy.append(self.startPointY)
        
        #print(theta)

        points = self.truckHead.get_xy()
        [x,y] = points[0]

        _x = self.startPointX + (math.cos(theta) * (x - oldX)) - (math.sin(theta) * (y - oldY))
        _y = self.startPointY + (math.sin(theta) * (x - oldX)) + (math.cos(theta) * (y - oldY))

        t = np.array([_x,_y])
        newPoints = np.array([t])

        for i in range(1,len(points)):
            [x,y] = points[i]
            _x = self.startPointX + (math.cos(theta) * (x - oldX)) - (math.sin(theta) * (y - oldY))
            _y = self.startPointY + (math.sin(theta) * (x - oldX)) + (math.cos(theta) * (y - oldY))
            t = np.array([_x,_y])
            newPoints = np.append(newPoints, [t], axis=0)

        self.truckHead.set_xy(newPoints)
        self.__updateTrailer(angleChange,oldX,oldY,dt)

    def __updateTrailer(self,th,oldX,oldY,dt):
        v = self.truckTrailer.get_xy()

        headTheta = np.radians(self.headAngle)
        gamma = np.radians(self.headAngle - self.trailerAngle)
        
        angleChange = -th * ((1/self.trailerW)*math.cos(gamma) + 1) - ((self.velocity/self.trailerW) * math.sin(gamma))
        #print(angleChange)
        self.trailerAngle += angleChange
        theta = np.radians(angleChange)

        x2 = self.startPointX# - (self.headW/2 * math.cos(headTheta)) - (self.trailerW * math.cos(theta))
        y2 = self.startPointY# - (self.headW/2 * math.sin(headTheta)) - (self.trailerW * math.sin(theta))
        self.save_tx.append(self.startPointX - (self.headW/2 * math.cos(math.radians(self.headAngle))) - (self.trailerW * math.cos(math.radians(self.trailerAngle))))
        self.save_ty.append(self.startPointY - (self.headW/2 * math.sin(math.radians(self.headAngle))) - (self.trailerW * math.sin(math.radians(self.trailerAngle))))

        [x,y] = v[0]
        #_x = x2 + math.cos(theta) * (x - self.oldTrailerPose['x']) - math.sin(theta) * (y - self.oldTrailerPose['y'])
        #_y = y2 + math.sin(theta) * (x - self.oldTrailerPose['x']) + math.cos(theta) * (y - self.oldTrailerPose['y'])
        _x = x2 + math.cos(theta) * (x - oldX) - math.sin(theta) * (y - oldY)
        _y = y2 + math.sin(theta) * (x - oldX) + math.cos(theta) * (y - oldY)
        
        t = np.array([_x,_y])
        newV = np.array([t])

        for i in range(1,len(v)):
            [x,y] = v[i]
            #_x = x2 + math.cos(theta) * (x - self.oldTrailerPose['x']) - math.sin(theta) * (y - self.oldTrailerPose['y'])
            #_y = y2 + math.sin(theta) * (x - self.oldTrailerPose['x']) + math.cos(theta) * (y - self.oldTrailerPose['y'])
            
            _x = x2 + math.cos(theta) * (x - oldX) - math.sin(theta) * (y - oldY)
            _y = y2 + math.sin(theta) * (x - oldX) + math.cos(theta) * (y - oldY)
            t = np.array([_x,_y])
            newV = np.append(newV, [t], axis=0)
        self.oldTrailerPose['x'] = x2
        self.oldTrailerPose['y'] = y2
        self.oldTrailerPose['theta'] = self.trailerAngle
        self.truckTrailer.set_xy(newV)