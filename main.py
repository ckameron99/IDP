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

    return xi, yi

def main():
    c = Camera()
    c.connect()

    cv2.imshow("normal", c.normalized)
    cv2.setMouseCallback('normal',c.mouseRGB)
    while True:
        cv2.imshow("normal", c.normalized)
        center = lineIntersection([c.getBarrier()], [c.getMainLine()])
        cv2.circle(c.normalized, center, radius=0, color=(0,0,255), thickness=1)
        # x1Max, y1Max, x2Min, y2Min = c.getBarrier()
        #cv2.line(c.normalized,(x1Max,y1Max),(x2Min,y2Min),(255,0,0),1)
        keya = cv2.waitKey(1)
        if keya == 27:
            exit()

if __name__ == "__main__":
    main()