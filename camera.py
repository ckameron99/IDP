import numpy as np
import cv2
import threading
import time


class Camera:
    def __init__(self):
        # Versions of frames are stored as a frame, followed by a number showing what frame it was derived from
        self.stream = None
        self._frame = None, 0
        self._normalized = None, -1
        self._ycrcb = None, -1
        self._ycrcbNormalized = None, -1
        self._hsv = None, -1
        self._cropped = None, -1
        self._arena = None, -1
        self._hsvArena = None, -1
        self.latestFrame = None
        with open("calibData.npz", "rb") as f:
            files = np.load(f, allow_pickle=True)
            self.ret = files["arr_0"]
            self.mtx = files["arr_1"]
            self.dist = files["arr_2"]
            self.rvecs = files["arr_3"]
            self.tvecs = files["arr_4"]
        self.t = threading.Thread(target=self.frameDaemon, daemon=True)

    def frameDaemon(self):
        while True:
            time.sleep(0.0001)
            ret = self.stream.grab()
            ret, frame = self.stream.retrieve()
            if ret:
                self.latestFrame = frame

    def connect(self, url="http://localhost:8082/stream/video.mjpeg"):
        self.stream = cv2.VideoCapture(url)
        self.t.start()
        time.sleep(1)

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

    def mouseHSV(self,event,x,y,flags,param):
        if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
            colorsH = self.hsvArena[y,x,0]
            colorsS = self.hsvArena[y,x,1]
            colorsV = self.hsvArena[y,x,2]
            colors = self.hsvArena[y,x]
            print("Hue: ",colorsH)
            print("Saturation: ",colorsS)
            print("Value: ",colorsV)
            print("HSV Format: ",colors)
            print("Coordinates of pixel: X: ",x,"Y: ",y)

    @property
    def frame(self):
        return self._frame[0]

    def updateFrame(self):
        '''ret = True
        while ret:
            ret = self.stream.grab()
        ret, frame = self.stream.retrieve()
        print("newframe")'''
        frame = self.latestFrame.copy()
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
    def hsv(self):
        if self._hsv[1] == self._frame[1]:
            return self._hsv[0]
        self._hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV), self._frame[1]
        return self._hsv[0]

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

    def four_point_transform(self, image, pts):
        # obtain a consistent order of the points and unpack them
        # individually
        pts = np.array(pts, dtype="float32")
        (tl, tr, br, bl) = pts

        # compute the width of the new image, which will be the
        # maximum distance between bottom-right and bottom-left
        # x-coordiates or the top-right and top-left x-coordinates
        widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
        widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
        maxWidth = max(int(widthA), int(widthB))

        # compute the height of the new image, which will be the
        # maximum distance between the top-right and bottom-right
        # y-coordinates or the top-left and bottom-left y-coordinates
        heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
        heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
        maxHeight = max(int(heightA), int(heightB))

        # now that we have the dimensions of the new image, construct
        # the set of destination points to obtain a "birds eye view",
        # (i.e. top-down view) of the image, again specifying points
        # in the top-left, top-right, bottom-right, and bottom-left
        # order
        dst = np.array([
            [0, 0],
            [maxWidth - 1, 0],
            [maxWidth - 1, maxHeight - 1],
            [0, maxHeight - 1]], dtype = "float32")

        # compute the perspective transform matrix and then apply it
        M = cv2.getPerspectiveTransform(pts, dst)
        t = time.time()
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        #print(f"warp: {time.time() -t}")
        warped = cv2.resize(warped, (600, 600), interpolation=cv2.INTER_AREA)
        #print(f"resize: {time.time()-t}")
        self._arena = warped, self._frame[1]

    @property
    def arena(self):
        return self._arena[0]

    @property
    def hsvArena(self):
        if self._hsvArena[1] == self._arena[1]:
            return self._hsvArena[0]
        self._hsvArena = cv2.cvtColor(self.arena, cv2.COLOR_BGR2HSV), self._arena[1]
        return self._hsvArena[0]

    def getBeacons(self):
        lower = np.uint8([33, 25, 175])
        upper = np.uint8([70, 85, 255])
        mask = cv2.inRange(self.hsvArena, lower, upper)
        
        kernal = np.ones((2,2), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal, iterations=2)

        try:
            points = cv2.findNonZero(mask).tolist()
        except:
            points = []

        groups = {}
        group = 0
        while len(points) > 1:
            group += 1
            groups[group] = []
            ref = points.pop(0)
            for i, point in enumerate(points):
                distance = ((point[0][0]-ref[0][0])**2 + (point[0][1]-ref[0][1])**2)**0.5
                if distance < 30:
                    groups[group].append(points[i][0])
                    points[i] = None
            points = list(filter(lambda x: x is not None, points))
        try:
            return list([[int(np.mean(list([x[0] for x in groups[arr]]))), int(np.mean(list([x[1] for x in groups[arr]])))] for arr in groups])
        except ValueError:
            return []

        #cv2.imshow("mask", mask)

    def getBarrier(self):
        return self.getBarrierhsv()

    def getBarrierhsv(self):
        lower = np.uint8([25, 50, 200])
        upper = np.uint8([35, 180, 255])
        yellowMask = cv2.inRange(self.hsv, lower, upper)

        #cv2.imshow("mask",yellowMask)

        
        kernal = np.ones((2,2), np.uint8)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernal, iterations=2)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernal, iterations=2)

        #cv2.imshow("mask2", yellowMask)

        lines = cv2.HoughLinesP(yellowMask,1,np.pi/500,30,minLineLength=120,maxLineGap=10)

        x1Max=0
        y1Max=0
        x2Min=10000
        y2Min=10000

        if lines is None or len(lines) == 0:
            return 0, 1, 0, 1

        for line in lines:
            x1Max = max(x1Max, line[0][0], line[0][2])
            y1Max = max(y1Max, line[0][1], line[0][3])
            x2Min = min(x2Min, line[0][2], line[0][0])
            y2Min = min(y2Min, line[0][3], line[0][1])

        return x1Max, y1Max, x2Min, y2Min
    
    def getBarrierycrcb(self):
        # no longer in use
        lower = np.uint8([130, 125, 35])
        upper = np.uint8([255, 145, 120])
        yellowMask = cv2.inRange(self.ycrcbNormalized, lower, upper)

        #cv2.imshow("mask",yellowMask)

        
        kernal = np.ones((2,2), np.uint8)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernal, iterations=2)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernal, iterations=2)

        #cv2.imshow("mask2", yellowMask)

        lines = cv2.HoughLinesP(yellowMask,1,np.pi/500,30,minLineLength=120,maxLineGap=10)

        x1Max=0
        y1Max=0
        x2Min=10000
        y2Min=10000

        if lines is None or len(lines) == 0:
            return 0, 1, 0, 1

        for line in lines:
            x1Max = max(x1Max, line[0][0], line[0][2])
            y1Max = max(y1Max, line[0][1], line[0][3])
            x2Min = min(x2Min, line[0][2], line[0][0])
            y2Min = min(y2Min, line[0][3], line[0][1])

        return x1Max, y1Max, x2Min, y2Min
            
    def getMainLine(self):
        lower = np.uint8([150, 125, 125])
        upper = np.uint8([255, 135, 135])
        whiteMask = cv2.inRange(self.ycrcbNormalized, lower, upper)
        
        kernal = np.ones((2,2), np.uint8)
        whiteMask = cv2.morphologyEx(whiteMask, cv2.MORPH_CLOSE, kernal, iterations=2)
        whiteMask = cv2.morphologyEx(whiteMask, cv2.MORPH_OPEN, kernal)

        lines = cv2.HoughLinesP(whiteMask,1,np.pi/180,30,minLineLength=200,maxLineGap=20)

        x1Max=0
        y1Max=0
        x2Min=10000
        y2Min=10000

        if lines is None:
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
        print(int(c.stream.get(cv2.CAP_PROP_FRAME_COUNT)))
        cv2.imshow("normal", c.normalized)
        x1Max, y1Max, x2Min, y2Min = c.getBarrier()
        #cv2.line(c.normalized,(x1Max,y1Max),(x2Min,y2Min),(255,0,0),1)
        keya = cv2.waitKey(1)
        if keya == 27:
            exit()

if __name__ == "__main__":
    main()