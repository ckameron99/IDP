from camera import Camera
import numpy as np
import cv2
import threading
import time

def gradient(l1):
    m1 = (l1[3] - l1[1]) / (l1[2] - l1[0])
    return m1

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

    return int(xi), int(yi)

def main():
    c = Camera()
    c.connect()
    c.updateFrame()
    cv2.imshow("normal", c.normalized)
    cv2.setMouseCallback('normal',c.mouseRGB)
    while True:
        c.updateFrame()
        barrier = c.getBarrier()
        #print(barrier)
        mainLine = c.getMainLine()
        center = lineIntersection(barrier, mainLine)
        #print(center)
        n = c.normalized.copy()
        cv2.circle(n, center, radius=0, color=(0,0,255), thickness=15)
        
        
        cv2.line(n,barrier[:2],barrier[2:],(255,0,0),5)
        cv2.line(n,mainLine[:2],mainLine[2:],(255,0,0),5)
        cv2.imshow("normal", n)
        
        keya = cv2.waitKey(1)
        if keya == 27:
            exit()

if __name__ == "__main__":
    main()