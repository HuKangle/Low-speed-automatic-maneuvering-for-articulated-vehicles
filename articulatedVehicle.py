import math
import numpy as np
import matplotlib.patches as patches


class ArticulatedVehicle:

    def __init__(self, plt, start_point_x, start_point_y):
        self.plt = plt
        self.velocity = 0
        self.vx = 0
        self.vy = 0
        self.trailerW = 65
        trailerH = 26
        self.headW = 30
        headH = 26
        self.startPointX = start_point_x
        self.startPointY = start_point_y
        self.headAngle = 0
        self.trailerAngle = 0
        self.max_v = 2
        self.max_th = 1
        self.save_hx = [self.startPointX]
        self.save_hy = [self.startPointY]
        self.save_tx = [self.startPointX - (self.headW / 2 * math.cos(np.radians(self.headAngle))) - (
                self.trailerW * math.cos(np.radians(self.trailerAngle)))]
        self.save_ty = [self.startPointY - (self.headW / 2 * math.sin(np.radians(self.headAngle))) - (
                self.trailerW * math.sin(np.radians(self.trailerAngle)))]

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
        p7 = [self.startPointX - hW, self.startPointY + tH]

        headVectors = np.array([p0, p1, p2, p3])
        trailerVectors = ([p4, p5, p6, p7])
        self.truckHead = patches.Polygon(headVectors, fill=None)
        self.truckTrailer = patches.Polygon(trailerVectors, fill=None)
        self.plt.gca().add_patch(self.truckHead)
        self.plt.gca().add_patch(self.truckTrailer)

    def move(self, vel, angle, dt):
        self.__updateHead(vel, angle, dt)

    def __updateHead(self, vel, angle, dt):
        self.velocity = vel

        angleChange = self.velocity / self.headW * math.tan(np.radians(angle))
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

        points = self.truckHead.get_xy()
        [x, y] = points[0]

        _x = self.startPointX + (math.cos(theta) * (x - oldX)) - (math.sin(theta) * (y - oldY))
        _y = self.startPointY + (math.sin(theta) * (x - oldX)) + (math.cos(theta) * (y - oldY))

        t = np.array([_x, _y])
        newPoints = np.array([t])

        for i in range(1, len(points)):
            [x, y] = points[i]
            _x = self.startPointX + (math.cos(theta) * (x - oldX)) - (math.sin(theta) * (y - oldY))
            _y = self.startPointY + (math.sin(theta) * (x - oldX)) + (math.cos(theta) * (y - oldY))
            t = np.array([_x, _y])
            newPoints = np.append(newPoints, [t], axis=0)

        self.truckHead.set_xy(newPoints)
       # self.plt.pause(0.01)
        self.__updateTrailer(angleChange, oldX, oldY, dt)

    def __updateTrailer(self, th, oldX, oldY, dt):
        v = self.truckTrailer.get_xy()
        gamma = np.radians(self.headAngle - self.trailerAngle)

        angleChange = -th * ((1 / self.trailerW) * math.cos(gamma) + 1) - (
                (self.velocity / self.trailerW) * math.sin(gamma))

        self.trailerAngle += angleChange
        theta = -np.radians(angleChange)

        hTheta = np.radians(self.headAngle)
        tTheta = -np.radians(self.trailerAngle)
        self.save_tx.append(self.startPointX - (self.headW / 2 * math.cos(hTheta)) - (self.trailerW * math.cos(tTheta)))
        self.save_ty.append(self.startPointY - (self.headW / 2 * math.sin(hTheta)) - (self.trailerW * math.sin(tTheta)))

        [x, y] = v[0]
        _x = self.startPointX + math.cos(theta) * (x - oldX) - math.sin(theta) * (y - oldY)
        _y = self.startPointY + math.sin(theta) * (x - oldX) + math.cos(theta) * (y - oldY)

        t = np.array([_x, _y])
        newV = np.array([t])

        for i in range(1, len(v)):
            [x, y] = v[i]

            _x = self.startPointX + math.cos(theta) * (x - oldX) - math.sin(theta) * (y - oldY)
            _y = self.startPointY + math.sin(theta) * (x - oldX) + math.cos(theta) * (y - oldY)
            t = np.array([_x, _y])
            newV = np.append(newV, [t], axis=0)

        self.truckTrailer.set_xy(newV)
       # self.plt.pause(0.01)

    def move_on_path(self, rx, ry):
        straight_movements = 0  # backward or forward
        turn_movements = 0  # right or left
        length = 0
        saveX = []
        saveY = []

        for i in range(len(rx) - 1):
            if rx[i + 1] > rx[i]:
                if ry[i + 1] == ry[i]:
                    print("X forward")
                    straight_movements += 1
                else:
                    print("turn 1")
                    turn_movements += 1
            elif rx[i + 1] == rx[i]:
                if ry[i + 1] == ry[i]:
                    print("P1 == P2")
                    continue
                elif ry[i + 1] > ry[i]:
                    print("Y forward")
                    straight_movements += 1
                else:
                    print("Y backward")
                    straight_movements += 1
            else:
                if ry[i + 1] == ry[i]:
                    print("X backward")
                    straight_movements += 1
                else:
                    # !!!! dangerous case !!!! **** collapse danger
                    print("turn dangerous")
                    turn_movements += 1

            angle, diagonal = self.find_the_angle(rx[i], ry[i], rx[i + 1], ry[i + 1])
            length += diagonal
            #self.loop_through_angle(angle, rx[i + 1], ry[i + 1])
            saveX.append(rx[i] + 20*np.cos(angle))
            saveY.append(ry[i] + 20*np.sin(angle))

        return straight_movements, turn_movements, length, saveX, saveY

    def loop_through_angle(self, angle, x_to_reach, y_to_reach):
        if angle == 0 and self.headAngle == 0:
            self.move(20, angle, 0.1)
            self.plt.pause(.0001)

        i = 0
        while self.headAngle < angle:
            i += 1
            self.move(20, i, 0.1)
            self.plt.pause(.0001)

        while self.startPointX < x_to_reach:
            self.move(20, 0, 0.1)

        while self.startPointY < y_to_reach:
            self.move(20, 0, 0.1)

    def find_the_angle(self, start_point_x, start_point_y, end_point_x, end_point_y):
        x = abs(end_point_x - start_point_x)
        y = abs(end_point_y - start_point_y)
        diagonal = math.sqrt((math.pow(x, 2) + math.pow(y, 2)))
        angle = math.acos(x / diagonal)
        adjust_to_the_head = abs(self.headAngle - np.rad2deg(angle))

        return adjust_to_the_head, diagonal
