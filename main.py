from camera import Camera
from robot import Robot
import numpy as np
import cv2
import threading
import time
from cv2 import aruco
import math
import time


def gradient(l1):
    if l1[2] == l1[0]:
        return 1
    m1 = (l1[3] - l1[1]) / (l1[2] - l1[0])
    return m1

def genLine(m, r):
    theta = math.atan2(1, m)
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return np.array([int(x), int(y)])

def genLine2(dir, r):
    theta = math.atan2(dir[1], dir[0])
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

    if m1 == m2:
        return 200

    xi = (c2 - c1) / (m1 - m2)
    yi = m1 * xi + c1

    return np.array([int(xi), int(yi)])

def timer(t1, t2):
    t=time.time()
    #print(t-t2)
    return t2, t

def main():
    t1, t2 = time.time(), time.time()
    r = Robot()
    r.connect("192.168.137.1", 8081, "", "")


    c = Camera()
    c.connect()
    #print("initializing")
    t1, t2 = timer(t1, t2)
    c.updateFrame()
    #cv2.setMouseCallback('normal',c.mouseRGB)
    arucoDict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = aruco.DetectorParameters_create()
    c.stream.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    c.updateFrame()
    barrier = c.getBarrier()
    mainLine = c.getMainLine()
    #print("got lines")
    t1, t2 = timer(t1, t2)
    center = lineIntersection(barrier, mainLine)
    corners = []
    barrierDir = [barrier[2]-barrier[0], barrier[3]-barrier[1]]
    mainLineDir = [mainLine[2]-mainLine[0], mainLine[3]-mainLine[1]]
    corners.append(center + genLine2(barrierDir, -350))
    corners.append(center + genLine2(mainLineDir, -350))
    corners.append(center + genLine2(barrierDir, 350))
    corners.append(center + genLine2(mainLineDir, 350))
    corners = np.array(corners)
    for i in range(10):
        c.updateFrame()
        barrier = c.getBarrier()
        mainLine = c.getMainLine()
        center = lineIntersection(barrier, mainLine)
        oldCorners = corners.copy()
        corners = []
        barrierDir = [barrier[2]-barrier[0], barrier[3]-barrier[1]]
        mainLineDir = [mainLine[2]-mainLine[0], mainLine[3]-mainLine[1]]
        corners.append(center + genLine2(barrierDir, -350))
        corners.append(center + genLine2(mainLineDir, -350))
        corners.append(center + genLine2(barrierDir, 350))
        corners.append(center + genLine2(mainLineDir, 350))
        corners = np.array(corners)
        corners = 0.2*corners + 0.8*oldCorners
        #print("got more lines")
        t1, t2 = timer(t1, t2)
    
    for i in range(1000):
        c.four_point_transform(c.frame, corners)
        beacons = c.getBeacons()
        r.setBeacons(beacons)
        if len(beacons) == 3:
            break
        if i == 1000:
            exit("failed to find beacons")
    framesLost = 0
    while True:
        t1, t2 = timer(t1, t2)
        c.updateFrame()
        #print("updated frame")
        t1, t2 = timer(t1, t2)
        c.four_point_transform(c.frame, corners)
        #print("transform")
        t1, t2 = timer(t1, t2)
        cv2.circle(c.arena, (300,300), radius=0, color=(0, 0, 255), thickness=5)
        cv2.circle(c.arena, (200,400), radius=0, color=(0, 0, 255), thickness=5)
        
        #print("found beacons")
        t1, t2 = timer(t1, t2)
        for beacon in beacons:
            cv2.circle(c.arena, beacon, radius=0, color=(255, 0, 0), thickness=5)
        

        
        for corner in corners:
            corner = [int(e) for e in corner]
            cv2.circle(c.frame, corner, radius=0, color=(0,255,0), thickness=5)
        #cv2.imshow("frame", c.frame)
        #print(center)
        #cv2.circle(c.frame, center, radius=0, color=(0,0,255), thickness=15)
        grey = cv2.cvtColor(c.arena, cv2.COLOR_BGR2GRAY)
        (rcorners, ids, rejected) = aruco.detectMarkers(c.arena[:,:,2], arucoDict, parameters=arucoParams)
        #print("detected aruco")
        t1, t2 = timer(t1, t2)
        if rcorners:
            framesLost = 0
            front = [
               int((rcorners[0][0][0][0] + rcorners[0][0][1][0]) / 2),
               int((rcorners[0][0][0][1] + rcorners[0][0][1][1]) / 2)
            ]
            back = [
                int((rcorners[0][0][2][0] + rcorners[0][0][3][0]) / 2),
                int((rcorners[0][0][2][1] + rcorners[0][0][3][1]) / 2)
            ]
            cv2.circle(c.arena, front, radius=0, color=(0,0,255), thickness=5)
            cv2.circle(c.arena, back, radius=0, color=(0,255,0), thickness=5)
            if time.time() - r.lastCommandTime > 0.3 and 0:
                print("calculating pos")
                r.computeMotors(front[0], front[1], back[0], back[1])
                print("computed motors")
                t1, t2 = timer(t1, t2)
        else:
            if framesLost < 10:
                if time.time() - r.lastCommandTime > 0.3:
                    r.commands.append("+000+000")
                    r.lastCommandTime = time.time()
                framesLost+=1
            else:
                print("nudge")
                r.commands.append("+250+250")
                r.lastCommandTime = time.time()
                framesLost = 0
        
        
        cv2.line(c.normalized,barrier[:2],barrier[2:],(255,0,0),5)
        cv2.line(c.normalized,mainLine[:2],mainLine[2:],(255,0,0),5)
        #cv2.imshow("frame", c.frame)
        cv2.imshow("normal", c.normalized)
        cv2.imshow("arena", c.arena)
        #cv2.imshow("original", c.stream.retrieve()[1])
        cv2.setMouseCallback("arena", c.mouseHSV)
        cv2.setMouseCallback("normal", c.mouseRGB)
        keya = cv2.waitKey(1)
        if keya == 27:
            r.commands.append("stop")
            r.lastCommandTime = time.time()
            cv2.destroyAllWindows()
            time.sleep(1)
            exit()
        if keya == 115:
            r.commands.append("start")
            r.lastCommandTime = time.time()


if __name__ == "__main__":
    main()