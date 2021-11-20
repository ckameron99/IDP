from camera import Camera
import numpy as np
import cv2
import threading
import time
from cv2 import aruco
import math

def gradient(l1):
    m1 = (l1[3] - l1[1]) / (l1[2] - l1[0])
    return m1

def genLine(m, r):
    theta = math.atan2(1, m)
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return np.array([int(x), int(y)])

def lineIntersection(l1, l2):
    # lines should be given in the format x1, y1, x2, y2
    # calculate the gradients
    m1 = gradient(l1)
    m2 = gradient(l2)

    # calculate the intercepts
    c1 = l1[1] - m1 * l1[0]
    c2 = l2[1] - m2 * l2[0]

    xi = (c2 - c1) / (m1 - m2)
    yi = m1 * xi + c1

    return np.array([int(xi), int(yi)])

def main():
    c = Camera()
    c.connect("sample2.mp4")
    c.updateFrame()
    #cv2.imshow("normal", c.normalized)
    #cv2.setMouseCallback('normal',c.mouseRGB)
    arucoDict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = aruco.DetectorParameters_create()
    c.stream.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    c.updateFrame()
    barrier = c.getBarrier()
    mainLine = c.getMainLine()
    center = lineIntersection(barrier, mainLine)
    corners = []
    corners.append(center + genLine(gradient(barrier), -350))
    corners.append(center + genLine(1/gradient(mainLine), -350))
    corners.append(center + genLine(gradient(barrier), 350))
    corners.append(center + genLine(1/gradient(mainLine), 350))
    corners = np.array(corners)
    while True:
        c.updateFrame()
        barrier = c.getBarrier()
        mainLine = c.getMainLine()
        center = lineIntersection(barrier, mainLine)
        oldCorners = corners.copy()
        corners = []
        corners.append(center + genLine(gradient(barrier), -350))
        corners.append(center + genLine(1/gradient(mainLine), -350))
        corners.append(center + genLine(gradient(barrier), 350))
        corners.append(center + genLine(1/gradient(mainLine), 350))
        corners = np.array(corners)
        corners = 0.2*corners + 0.8*oldCorners

        c.four_point_transform(c.frame, corners)
        cv2.circle(c.arena, (300,300), radius=0, color=(0, 0, 255), thickness=5)
        cv2.circle(c.arena, (200,400), radius=0, color=(0, 0, 255), thickness=5)
        cv2.imshow("arena", c.arena)
        '''for corner in corners:
            cv2.circle(c.frame, corner, radius=0, color=(0,255,0), thickness=5)'''
        #print(center)
        #cv2.circle(c.frame, center, radius=0, color=(0,0,255), thickness=15)
        grey = cv2.cvtColor(c.arena, cv2.COLOR_BGR2GRAY)
        (rcorners, ids, rejected) = aruco.detectMarkers(grey, arucoDict, parameters=arucoParams)
        if rcorners:
            cv2.circle(c.arena, (int(rcorners[0][0][0][0]),int(rcorners[0][0][0][1])), radius=0, color=(0,0,255), thickness=5)
            cv2.circle(c.arena, (int(rcorners[0][0][1][0]),int(rcorners[0][0][1][1])), radius=0, color=(0,0,255), thickness=5)
            cv2.circle(c.arena, (int(rcorners[0][0][2][0]),int(rcorners[0][0][2][1])), radius=0, color=(0,0,255), thickness=5)
            cv2.circle(c.arena, (int(rcorners[0][0][3][0]),int(rcorners[0][0][3][1])), radius=0, color=(0,0,255), thickness=5)
        
        
        cv2.line(c.normalized,barrier[:2],barrier[2:],(255,0,0),5)
        cv2.line(c.normalized,mainLine[:2],mainLine[2:],(255,0,0),5)
        #cv2.imshow("frame", c.frame)
        #cv2.imshow("normal", c.normalized)
        
        keya = cv2.waitKey(1)
        if keya == 27:
            exit()

if __name__ == "__main__":
    main()