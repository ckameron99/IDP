import numpy as np
import cv2
import threading
import time


class Camera:
    def __init__(self):
        # Versions of frames are stored as a frame, followed by a number showing what frame it was derived from
        self._frame = None, 0
        self._normalized = None, -1
        self._ycrcb = None, -1
        self._ycrcbNormalized = None, -1
        with open("calibData.npz", "rb") as f:
            files = np.load(f, allow_pickle=True)
            self.ret = files["arr_0"]
            self.mtx = files["arr_1"]
            self.dist = files["arr_2"]
            self.rvecs = files["arr_3"]
            self.tvecs = files["arr_4"]

    def connect(self, url="http://localhost:8081/stream/video.mjpeg"):
        self.stream = cv2.VideoCapture(url)

    def mouseRGB(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
            colorsB = self._normalized[0][y,x,0]
            colorsG = self._normalized[0][y,x,1]
            colorsR = self._normalized[0][y,x,2]
            colors = self._normalized[0][y,x]
            ycrcb = cv2.cvtColor(self.normalized, cv2.COLOR_BGR2YCrCb)
            print("ycrcb: ", ycrcb[y,x])
            print("Red: ",colorsR)
            print("Green: ",colorsG)
            print("Blue: ",colorsB)
            print("BRG Format: ",colors)
            print("Coordinates of pixel: X: ",x,"Y: ",y)

    @property
    def frame(self):
        return self._frame[0]

    def updateFrame(self):
        ret, frame = self.stream.read()
        if ret == True:
            h,  w = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
            dst = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)
            x, y, w, h = roi
            dst = dst[y-50:y+h+50, x:x+w]
            self._frame = dst, self._frame[1] + 1
        return self._frame[0]

    @frame.setter
    def frame(self, value):
        self._frame[0] = value

    @property
    def ycrcb(self):
        if self._ycrcb[1] == self._frame[1]:
            return self._ycrcb[0]
        self._ycrcb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2YCrCb), self._frame[1]
        return self._ycrcb[0]

    @property
    def ycrcbNormalized(self):
        hh, ww = self.frame.shape[:2]
        if self._normalized[1] == self._frame[1]:
            return self._normalized[0]
        mx = max(hh, ww)

        y, cr, cb = cv2.split(self.ycrcb)

        sigma = int(5 * mx / 300)

        gaussian = cv2.GaussianBlur(y, (0, 0), sigma, sigma)

        y = y.astype("float32")
        gaussian = gaussian.astype("float32")

        y = y - gaussian + 100

        y = np.clip(y, 0, 255)

        y = y.astype("uint8")

        self._ycrcbNormalized = cv2.merge([y, cr, cb]), self._frame[1]
        return self._ycrcbNormalized[0]

    @property
    def normalized(self):
        hh, ww = self.frame.shape[:2]
        if self._normalized[1] == self._frame[1]:
            return self._normalized[0]

        self._normalized = cv2.cvtColor(self.ycrcbNormalized, cv2.COLOR_YCR_CB2BGR), self._frame[1]

        return self._normalized[0]

    @normalized.setter
    def normalized(self, value):
        self._normalized = value

    def getBarrier(self):
        lower = np.uint8([130, 125, 50])
        upper = np.uint8([255, 140, 120])
        yellowMask = cv2.inRange(self.ycrcbNormalized, lower, upper)

        
        kernal = np.ones((2,2), np.uint8)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernal, iterations=2)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernal, iterations=2)

        cv2.imshow("mask",yellowMask)
        cv2.waitKey(1)

        lines = cv2.HoughLinesP(yellowMask,1,np.pi/500,30,minLineLength=120,maxLineGap=10)

        x1Max=0
        y1Max=0
        x2Min=10000
        y2Min=10000

        if len(lines) == 0:
            return 0, 1, 0, 1

        for line in lines:
            x1Max = max(x1Max, line[0][0], line[0][2])
            y1Max = max(y1Max, line[0][1], line[0][3])
            x2Min = min(x2Min, line[0][2], line[0][0])
            y2Min = min(y2Min, line[0][3], line[0][1])

        return x1Max, y1Max, x2Min, y2Min
            
    def getMainLine(self):
        lower = np.uint8([200, 200, 200])
        upper = np.uint8([255, 255, 255])
        whiteMask = cv2.inRange(self.normalized, lower, upper)

        lower = np.uint8([150, 150, 150])
        upper = np.uint8([200, 200, 200])
        darkWhiteMask = cv2.inRange(self.normalized, lower, upper)

        whiteMask = whiteMask | darkWhiteMask
        
        kernal = np.ones((2,2), np.uint8)
        whiteMask = cv2.morphologyEx(whiteMask, cv2.MORPH_CLOSE, kernal, iterations=2)
        whiteMask = cv2.morphologyEx(whiteMask, cv2.MORPH_OPEN, kernal)

        lines = cv2.HoughLinesP(whiteMask,1,np.pi/180,30,minLineLength=200,maxLineGap=20)

        x1Max=0
        y1Max=0
        x2Min=10000
        y2Min=10000

        if len(lines) == 0:
            return 0, 1, 0, 1

        for line in lines:
            x1Max = max(x1Max, line[0][0], line[0][2])
            y1Max = max(y1Max, line[0][1], line[0][3])
            x2Min = min(x2Min, line[0][2], line[0][0])
            y2Min = min(y2Min, line[0][3], line[0][1])

        return x1Max, y2Min, x2Min, y1Max


def main():
    c = Camera()
    c.connect()

    cv2.imshow("normal", c.normalized)
    cv2.setMouseCallback('normal',c.mouseRGB)
    while True:
        cv2.imshow("normal", c.normalized)
        x1Max, y1Max, x2Min, y2Min = c.getBarrier()
        #cv2.line(c.normalized,(x1Max,y1Max),(x2Min,y2Min),(255,0,0),1)
        keya = cv2.waitKey(1)
        if keya == 27:
            exit()

if __name__ == "__main__":
    main()