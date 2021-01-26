# This Python file uses the following encoding: utf-8
#Made by Nanok
import numpy as np
import math


class Bot:

    ALPHA = 0
    D = 1
    THETA = 2
    R = 3

    def __init__(self):

        self.params = np.array([])
        self.l = 0

        self.ta = 0
        self.tb = 0

    
    def init(self, l = [3.5, 3, 3], theta = [0, 55, 75]):

        self.params = np.array([])

        self.l = l[len(l) - 1]

        for i, v in enumerate(l):

            if i == len(l) - 1:
                break

            #Pour notre robot, les paramètre alpha et r son tous nuls
            row = np.array([0, v, math.radians(theta[i + 1]), 0])

            if self.params.size != 0:
                self.params = np.vstack((self.params, row))
                continue

            self.params = row


    def getPmatrice(self, f, t):
        j = f-1
        alpha = self.params[j][self.ALPHA]
        theta = self.params[j][self.THETA]
        return np.array([
            [math.cos(theta), -math.sin(theta), 0, self.params[j][self.D]],
            [math.cos(alpha) * math.sin(theta), math.cos(alpha) * math.cos(theta), -math.sin(alpha), -math.sin(alpha) * self.params[j][self.R]],
            [math.sin(alpha) * math.sin(theta), math.sin(alpha) * math.cos(theta), -math.cos(alpha), math.cos(alpha) * self.params[j][self.R]],
            [0, 0, 0, 1]
        ])

    def getPimatrice(self, f, t):
        j = f
        alpha = self.params[j][self.ALPHA]
        theta = self.params[j][self.THETA]
        return np.array([
            [math.cos(theta), math.cos(alpha) * math.sin(theta), math.sin(alpha) * math.sin(theta), -self.params[j][self.D] * math.cos(theta)],
            [-math.sin(theta), math.cos(alpha) * math.cos(theta), math.sin(alpha) * math.cos(theta), self.params[j][self.D] * math.sin(theta)],
            [0, -math.sin(alpha), math.cos(alpha), -self.params[j][self.R]],
            [0, 0, 0, 1]
        ])


    def getPositionA(self):
        a = np.array([self.l, 0, 0, 1])

        pm = np.dot(self.getPmatrice(1, 0), self.getPmatrice(2, 1))
        a = np.dot(pm, a.T)

        return a

    def eqt2(self, x, y, z):
        if z == 0:
            return math.atan2(-y, x)
        else:
            return [
                math.atan2((z * x + y * math.sqrt(x * x + y * y - z * z)) / (x * x + y * y), (z * y - x * math.sqrt(x * x + y * y - z * z)) / (x * x + y * y)), #epsilon = 1
                math.atan2((z * x - y * math.sqrt(x * x + y * y - z * z)) / (x * x + y * y), (z * y + x * math.sqrt(x * x + y * y - z * z)) / (x * x + y * y)) #epsilon = -1
            ]


    def eqt3(self, x1, x2, y1, y2):
        return math.atan2(y1 / x1, y2 / x2)

    def eqt61(self, w, x, y, z1, z2):
        return [2 * (y * z1 + x * z2), 2 * (x * z1 - y * z2), w * w - x * x - y * y - z1 * z1 - z2 * z2]

    def eqt62(self, w, x, y, z1, z2, theta):
        return [w, w, x * math.cos(theta) + y * math.sin(theta) + z1, x * math.sin(theta) - y * math.cos(theta) + z2]

    def mgi(self, b):

        #Pour j = 1

        w = self.l
        x = b[1]
        y = self.params[0][self.D] - b[0]
        z1 = 0
        z2 = -self.params[1][self.D]

        returned_b = self.eqt61(w, x, y, z1, z2)

        index = 0

        theta1 = self.eqt2(returned_b[0], returned_b[1], returned_b[2])
        theta2 = []
        for i, v in enumerate(theta1):
            returned_xy = self.eqt62(w, x, y, z1, z2, v)
            theta2.append(self.eqt3(returned_xy[0], returned_xy[1], returned_xy[2], returned_xy[3]))

            if v * theta2[i] >= 0 and v >= 0:
                index = i 

        #remplace les angles theta par leurs nouvelles valeurs
        
        self.params[0][self.THETA] = theta1[index]
        self.params[1][self.THETA] = theta2[index]


    def eqTcoord(self, a, b):
        self.ta = (b[1] - a[1]) / (b[0] - a[0])
        self.tb = b[1] - self.ta * b[0]

    def eqT(self, x):
        return self.ta * x + self.tb

    def getStep(self, a, b, nb):
        return (b[0] - a[0]) / nb


