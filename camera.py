import numpy as np
import cv2
import threading
import time


class Camera:
    """A class to contain properties and functions related to the camera and it's stream"""
    def __init__(self):
        """Initializes the camera class, including several placeholders for transformed frames"""
        self.stream = None  # placeholder for the stream object
        self._frame = None, 0  # placeholder for current frame being worked on, as well as a sequential numeric identifier
        self._normalized = None, -1  # placeholder for a brightness normalized frame, as well as a sequential numeric identifier
        self._ycrcb = None, -1  # placeholder for the frame in a ycrcb colourspace, as well as a sequential numeric identifier
        self._ycrcbNormalized = None, -1  # placeholder for the normalized frame in a ycrcb colourspace, as well as a sequential numeric identifier
        self._hsv = None, -1  # placeholder for the frame in a HSV colourspace, as well as a sequential numeric identifier
        self._arena = None, -1  # placeholder for the arena extracted from the frame, as well as a sequential numeric identifier
        self._hsvArena = None, -1  # placeholder for the arena in a HSV colourspace, as well as a sequential numeric identifier
        self.latestFrame = None  # stores the latest frame from the stream
        with open("calibData.npz", "rb") as f:
            # load the calibration data from a serialized file.
            files = np.load(f, allow_pickle=True)
            self.ret = files["arr_0"]
            self.mtx = files["arr_1"]
            self.dist = files["arr_2"]
            self.rvecs = files["arr_3"]
            self.tvecs = files["arr_4"]
        self.t = threading.Thread(target=self.frameDaemon, daemon=True)

    def frameDaemon(self):
        """Daemon to clear the frame buffer and stores the latest frame"""
        while True:
            time.sleep(0.0001)
            ret = self.stream.grab()  # get a new frame
            ret, frame = self.stream.retrieve()  # retrieve the new frame
            if ret:
                self.latestFrame = frame  # if there was a new frame, update the latest frame to match

    def connect(self, url="http://localhost:8082/stream/video.mjpeg"):
        """Starts the stream and the frame buffer daemon"""
        self.stream = cv2.VideoCapture(url)
        self.t.start()  # begin the frame updating daemon
        time.sleep(1)

    def mouseRGB(self,event,x,y,flags,param):
        """Callback event for clicking on an RGB display"""
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
        """Callback event for clicking on an HSV display"""
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
        """Getter method for the frame property"""
        return self._frame[0]

    def updateFrame(self):
        """Updates the frame from the latest frame from stream daemon"""
        frame = self.latestFrame.copy()  # make a copy of the latest frame
        h, w = frame.shape[:2]  # extract the height and width

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
        dst = cv2.undistort(frame, self.mtx, self.dist, None, newcameramtx)
        # undistort the image

        x, y, w, h = roi
        dst = dst[y-50:y+h+50, x:x+w]
        # crop to the arena plus some bits

        self._frame = dst, self._frame[1] + 1
        return self._frame[0]

    @frame.setter
    def frame(self, value):
        """Setter method for the frame property. Does NOT update the frame number"""
        self._frame[0] = value

    @property
    def hsv(self):
        """Getter method for the HSV property. TODO: use @cached_property"""
        if self._hsv[1] == self._frame[1]:
            return self._hsv[0]
        self._hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV), self._frame[1]
        return self._hsv[0]

    @property
    def ycrcb(self):
        """Getter method for the ycrcb property. TODO: use @cached_property"""
        if self._ycrcb[1] == self._frame[1]:
            return self._ycrcb[0]
        self._ycrcb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2YCrCb), self._frame[1]
        return self._ycrcb[0]

    @property
    def ycrcbNormalized(self):
        """Getter method for the ycrcbNormalized property.
        Normalization pulls the avarage of local areas to a brightness of 100.
        TODO: use @cached_property"""
        hh, ww = self.frame.shape[:2]
        if self._normalized[1] == self._frame[1]:
            return self._normalized[0]
        mx = max(hh, ww)  # get maximum dimension

        y, cr, cb = cv2.split(self.ycrcb)  # split into each colour channel

        sigma = int(5 * mx / 300)  # how much to blur - proportional to the largest dimension

        gaussian = cv2.GaussianBlur(y, (0, 0), sigma, sigma)  # apply a Gaussian blur

        y = y.astype("float32")  # convert the dtype to float32 to prevent overflows
        gaussian = gaussian.astype("float32")  # convert the dtype to float32 to prevent overflows

        y = y - gaussian + 100  # normalize the brightness to 100

        y = np.clip(y, 0, 255)  # cap the brightness and darkness to max values

        y = y.astype("uint8")  # change dtype back to uint8

        self._ycrcbNormalized = cv2.merge([y, cr, cb]), self._frame[1]  # combine colour channels back, and update frame number
        return self._ycrcbNormalized[0]

    @property
    def normalized(self):
        """Getter method for the normalized property. TODO: use @cached_property"""
        hh, ww = self.frame.shape[:2]
        if self._normalized[1] == self._frame[1]:
            return self._normalized[0]

        self._normalized = cv2.cvtColor(self.ycrcbNormalized, cv2.COLOR_YCR_CB2BGR), self._frame[1]  # convert back to BGR colourspace

        return self._normalized[0]

    @normalized.setter
    def normalized(self, value):
        """Setter method for the normalized property"""
        self._normalized = value

    def four_point_transform(self, image, pts):
        """Given and image and the vertices of a quaderalateral, sets the arena
        property to a square frame containing the same area as the quaderalateral.
        This function is widely used online but appears to have originated in the pyimagesearch library, and was modified for our use."""
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
        warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
        warped = cv2.resize(warped, (600, 600), interpolation=cv2.INTER_AREA)
        self._arena = warped, self._frame[1]

    @property
    def arena(self):
        """Getter method for the arena property"""
        return self._arena[0]

    @property
    def hsvArena(self):
        """Getter method for the hsvArena property. TODO: use @cached_property."""
        if self._hsvArena[1] == self._arena[1]:
            return self._hsvArena[0]
        self._hsvArena = cv2.cvtColor(self.arena, cv2.COLOR_BGR2HSV), self._arena[1]
        return self._hsvArena[0]

    def getBeacons(self):
        """Returns a list of the beacons."""
        lower = np.uint8([34, 25, 175])
        upper = np.uint8([70, 85, 255])
        mask = cv2.inRange(self.hsvArena, lower, upper)
        # create a mask of pixles that match the colour of the beacons in the HSV colourspace
        
        kernel = np.ones((2,2), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        # close and open mask to reduce noise

        try:
            points = cv2.findNonZero(mask).tolist()  # create a list of all points in the mask
        except Exception:
            points = []

        groups = {}  # create a blank index of nearby points
        group = 0
        while len(points) > 1:  # i.e. until all points from mask have been grouped
            group += 1  # create a new group number
            groups[group] = []
            ref = points.pop(0)  # root point
            for i, point in enumerate(points):
                distance = ((point[0][0]-ref[0][0])**2 + (point[0][1]-ref[0][1])**2)**0.5
                if distance < 30:
                    groups[group].append(points[i][0])
                    points[i] = None
                # if a point is within 30 pixels to the root point, remove it from the list and add it to the group

            points = list(filter(lambda x: x is not None, points))
            # remove none points
        try:
            return list([[int(np.mean(list([x[0] for x in groups[arr]]))), int(np.mean(list([x[1] for x in groups[arr]])))] for arr in groups])
        except ValueError:
            return []

    def getBarrier(self):
        """Allows for either using an HSV mask or ycrcb mask"""
        return self.getBarrierhsv()

    def getBarrierhsv(self):
        """Returns the points defining the barrier. Uses an HSV mask"""
        lower = np.uint8([25, 50, 200])
        upper = np.uint8([35, 180, 255])
        yellowMask = cv2.inRange(self.hsv, lower, upper)
        # create a mask of pixles that match the colour of the beacons in the HSV colourspace

        
        kernel = np.ones((2,2), np.uint8)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernel, iterations=2)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernel, iterations=2)
        # close and open mask to reduce noise

        lines = cv2.HoughLinesP(yellowMask,1,np.pi/500,30,minLineLength=120,maxLineGap=10)
        # extract many possible lines

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
        # find minimum and maximum values of x and y coordinates of all lines

        return x1Max, y1Max, x2Min, y2Min
    
    def getBarrierycrcb(self):
        """Returns the points defining the barrier. Uses a ycrcb mask."""
        # no longer in use
        lower = np.uint8([130, 125, 35])
        upper = np.uint8([255, 145, 120])
        yellowMask = cv2.inRange(self.ycrcbNormalized, lower, upper)
        # create a mask for the barrier based on the ycrcb colourspace

        
        kernel = np.ones((2,2), np.uint8)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_CLOSE, kernel, iterations=2)
        yellowMask = cv2.morphologyEx(yellowMask, cv2.MORPH_OPEN, kernel, iterations=2)
        # close and open the mask to reduce noise

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
        # find minimum and maximum values of x and y coordinates of all lines

        return x1Max, y1Max, x2Min, y2Min
            
    def getMainLine(self):
        """Returns the coordinates of the ends of the main white line"""
        lower = np.uint8([150, 125, 125])
        upper = np.uint8([255, 135, 135])
        whiteMask = cv2.inRange(self.ycrcbNormalized, lower, upper)
        
        kernel = np.ones((2,2), np.uint8)
        whiteMask = cv2.morphologyEx(whiteMask, cv2.MORPH_CLOSE, kernel, iterations=2)
        whiteMask = cv2.morphologyEx(whiteMask, cv2.MORPH_OPEN, kernel)

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
        # find minimum and maximum values of x and y coordinates of all lines

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