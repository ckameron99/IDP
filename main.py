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
    """Returns the gradient of a line in the form of the coordinates of each end"""
    if l1[2] == l1[0]:
        return 1
    m1 = (l1[3] - l1[1]) / (l1[2] - l1[0])
    return m1

def genLine(m, r):
    """Generate a line segment from the gradient and the length"""
    theta = math.atan2(1, m)
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return np.array([int(x), int(y)])

def genLine2(dir, r):
    """Generate a line segment from a direction and length"""
    theta = math.atan2(dir[1], dir[0])
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return np.array([int(x), int(y)])

def lineIntersection(l1, l2):
    """Calculate the location of the intersection of two lines."""
    # lines should be given in the format x1, y1, x2, y2
    # calculate the gradients
    m1 = gradient(l1)
    m2 = gradient(l2)

    # calculate the intercepts with the y axis
    c1 = l1[1] - m1 * l1[0]
    c2 = l2[1] - m2 * l2[0]

    if m1 == m2:
        return 200

    xi = (c2 - c1) / (m1 - m2)
    yi = m1 * xi + c1

    return np.array([int(xi), int(yi)])


def main():
    """Main program loop."""
    r = Robot()
    r.connect("192.168.137.1", 8081)
    # create and connect robot instance


    c = Camera()
    c.connect()
    # create and connect camera instance
    
    c.updateFrame()
    #cv2.setMouseCallback('normal',c.mouseRGB)

    arucoDict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    arucoParams = aruco.DetectorParameters_create()
    # set up the arUco code dictionary and parameters for later recognition

    c.stream.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # an attempt to reduce latency, redundant after frame daemon implemented.
    c.updateFrame()

    barrier = c.getBarrier()
    mainLine = c.getMainLine()

    center = lineIntersection(barrier, mainLine)
    # calculate center of the arena
    
    barrierDir = [barrier[2]-barrier[0], barrier[3]-barrier[1]]
    mainLineDir = [mainLine[2]-mainLine[0], mainLine[3]-mainLine[1]]
    # calculate the direction the barrier and main line travel along

    corners = []
    corners.append(center + genLine2(barrierDir, -350))
    corners.append(center + genLine2(mainLineDir, -350))
    corners.append(center + genLine2(barrierDir, 350))
    corners.append(center + genLine2(mainLineDir, 350))
    corners = np.array(corners)
    # estimate the corners of the arena from the directions and center
    
    for i in range(10):
        c.updateFrame()

        barrier = c.getBarrier()
        mainLine = c.getMainLine()
        # get the lines of the main line and barrier

        center = lineIntersection(barrier, mainLine)
        # calculate the center of the arena

        oldCorners = corners.copy() # store a copy of the current corner estimate

        barrierDir = [barrier[2]-barrier[0], barrier[3]-barrier[1]]
        mainLineDir = [mainLine[2]-mainLine[0], mainLine[3]-mainLine[1]]
        # calculate the direction the main line and barrier point in

        corners = []
        corners.append(center + genLine2(barrierDir, -350))
        corners.append(center + genLine2(mainLineDir, -350))
        corners.append(center + genLine2(barrierDir, 350))
        corners.append(center + genLine2(mainLineDir, 350))
        corners = np.array(corners)
        # estimate the corners of the arena from the directions and center

        corners = 0.2*corners + 0.8*oldCorners
        # perform a weighted avarage
    
    for i in range(1000):
        try:
            corners = corners.tolist()
        except Exception:
            pass
        cornersNew = corners[::-1]  # reverse list
        cornersNew = [
            corners[2], corners[1], corners[0], corners[3]
        ]
        # reorder list

        c.four_point_transform(c.frame, cornersNew)
        # create cropped arena

        beacons = c.getBeacons()
        r.setBeacons(beacons)
        # detect beacons in current frame and update robot's model

        if len(beacons) == 3:
            break  # assume correct recognition to break
        if i == 1000:
            exit("failed to find beacons")

    framesLost = 0  # used for relocation of robot if image recognition fails
    while True:
        c.updateFrame()
        try:
            corners = corners.tolist()
        except Exception:
            pass
        # corners may be either a list or ndarray. Make it a list
        cornersNew = corners[::-1]  # reverse list
        cornersNew = [
            corners[2], corners[1], corners[0], corners[3]
        ]
        # reorder list

        c.four_point_transform(c.frame, cornersNew)
        # create cropped arena frame

        cv2.circle(c.arena, (300,300), radius=0, color=(0, 0, 255), thickness=5)
        cv2.circle(c.arena, (200,400), radius=0, color=(0, 0, 255), thickness=5)
        cv2.circle(c.arena, (400,200), radius=0, color=(0, 0, 255), thickness=5)
        # plot some circles at key points in arena to ensure correct transform

        for beacon in beacons:
            cv2.circle(c.arena, beacon, radius=0, color=(255, 0, 0), thickness=5)
        # plot beacons

        
        for corner in corners:
            corner = [int(e) for e in corner]
            cv2.circle(c.frame, corner, radius=0, color=(0,255,0), thickness=5)
        # plot the corners of the arena

        (rcorners, ids, rejected) = aruco.detectMarkers(c.arena[:,:,2], arucoDict, parameters=arucoParams)
        # detect the arUco markers

        if rcorners:
            framesLost = 0  # reset the unrecognized frame counter
            front = [
               int((rcorners[0][0][0][0] + rcorners[0][0][1][0]) / 2),
               int((rcorners[0][0][0][1] + rcorners[0][0][1][1]) / 2)
            ]
            back = [
                int((rcorners[0][0][2][0] + rcorners[0][0][3][0]) / 2),
                int((rcorners[0][0][2][1] + rcorners[0][0][3][1]) / 2)
            ]
            # calculate the coordinates of the front and back of the arUco code

            cv2.circle(c.arena, front, radius=0, color=(0,0,255), thickness=5)
            cv2.circle(c.arena, back, radius=0, color=(0,255,0), thickness=5)
            # plot the front and back of the robot

            if time.time() - r.lastCommandTime > 0.15:  # prevent overloading robot with commands
                r.computeMotors(front[0], front[1], back[0], back[1])  # get the robot to move
            else:
                print("skipped")
        else:
            if framesLost < 10:
                if time.time() - r.lastCommandTime > 0.15:
                    r.commands.append("+000+000")
                    r.lastCommandTime = time.time()
                framesLost+=1
                # if the robot is recently lost, stop it from moving
            else:
                print("nudge")
                r.commands.append("+250+250")
                r.lastCommandTime = time.time()
                framesLost = 0
                # if the robot has been lost for a while, nudge it forwards a little to see if the lighting changes
        
        
        cv2.line(c.normalized,barrier[:2],barrier[2:],(255,0,0),5)
        cv2.line(c.normalized,mainLine[:2],mainLine[2:],(255,0,0),5)
        # plot the barrier and main line over the normalized view
        
        cv2.imshow("normal", c.normalized)
        cv2.imshow("arena", c.arena)
        # display the normalized view and arena view

        cv2.setMouseCallback("arena", c.mouseHSV)
        cv2.setMouseCallback("normal", c.mouseRGB)
        # add mouse callback events for debugging

        keya = cv2.waitKey(1)
        # render a frame and listen for a keypress

        if keya == 27:  # 'Esc' key
            r.commands.append("stop")
            r.lastCommandTime = time.time()
            cv2.destroyAllWindows()
            time.sleep(1)
            exit()
        if keya == 115:  # 's' key
            r.commands.append("start")
            r.lastCommandTime = time.time()
            print("start")


if __name__ == "__main__":
    main()