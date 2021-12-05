import threading
import socket
import math
import time
import atexit


class Robot:
    """Class to contain methods and properties relating to the physical robot."""
    def __init__(self):
        """Initializes the robot class"""
        self.commands = []
        self.beacons = []
        self.dropOffLocations = {
            "1": (80, 520),
            "2": (450, 200),
            "3": (400, 150),
            "0": (565, 45)
        }
        self.returning = False
        self.beaconID = None
    
    def connect(self, IPAddr, IPPort):
        """Starts listening for the robot to connect, and begins a daemon to send commands. TODO: implement bluetooth."""
        #self.bts = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.ips = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        #self.bts.bind((socket.BDADDR_ANY, 0))
        self.ips.bind((IPAddr, IPPort))

        #self.bts.listen()
        self.ips.listen()

        atexit.register(self.ips.close)
        
        self.t = threading.Thread(target=self.IPDaemon, daemon=True)
        self.t.start()

        self.commands = []

        self.lastCommandTime = 0

    def IPDaemon(self):
        """Daemon to send buffered commands to the robot"""
        while True:
            conn, addr = self.ips.accept()
            self.commands = []
            with conn:
                d = conn.recv(1024)  # should be b'test'
                print(d)
                while True:
                    if self.commands != []:
                        conn.sendall(self.commands.pop(0).encode("utf-8"))
                    time.sleep(0.001)

    def computeMotors(self, x1, y1, x2, y2):
        """Given the location of the front and back of the arUco code, calculate what the motors should be doing"""
        xa = (x1 + x2) / 2
        ya = (y1 + y2) / 2
        position = (xa, ya)
        print("computing motors")
        print(f"beacons: {self.beacons}")
        i = 0
        for j, beacon in enumerate(self.beacons):
            if abs(sum(beacon) - 600) < 40:
                i = j
        if i == len(self.beacons):
            print("all beacons on other side")
            return

        orientation = [y2 - y1, x2 - x1]
        reversing = False

        if self.returning:
            reversing = True
            instrumentalDestination, finalDestination = (175,425), (175,425)
            print(instrumentalDestination)
        elif len(self.beacons) == 0:
            finalDestination = self.dropOffLocations["0"]
            print(f"final destination: {finalDestination}")
            print(xa, ya)
            if ((xa-300)**2 + (ya-300)**2)**0.5 < 220 and abs(xa+ya-600)<40 and ya - xa < 220:
                instrumentalDestination = (175,425)
            elif xa - ya > 0:
                instrumentalDestination = (425,175)
            else:
                instrumentalDestination = finalDestination
            print(instrumentalDestination)
        else:
            finalDestination = self.beacons[i]
            print(f"final destination: {finalDestination}")
            print(xa, ya)
            if ((xa-300)**2 + (ya-300)**2)**0.5 < 220 and abs(xa+ya-600)<40 and ya - xa < 220:
                instrumentalDestination = (175,425)
            elif xa - ya > 0:
                instrumentalDestination = (425,175)
            else:
                instrumentalDestination = finalDestination
            print(instrumentalDestination)
            

        

            
        print(f"orientation: {orientation}")
        direction = [ya - instrumentalDestination[1], xa - instrumentalDestination[0]]
        print(f"direction: {direction}")

        directionToFinal = [ya - finalDestination[1], xa - finalDestination[0]]

        distance = (directionToFinal[0]**2 + directionToFinal[1]**2)**0.5

        if distance < 95 and not self.returning:
            print("begun approach")
            self.commands.append("approach")
            self.lastCommandTime = time.time()
            self.beacons.pop(i)
            self.returning = True
            return
        elif distance < 50:
            self.returning = False
            if len(self.beacons) == 0:
                self.commands.append("+000+000")
                exit()
            return

        error = math.atan2(*direction) - math.atan2(*orientation)
        print(error)
        lPower = 255 + error*500
        rPower = 255 - error*500
        maxPower = max(abs(lPower), abs(rPower))
        lPower /= maxPower/255
        rPower /= maxPower/255
        lPower = int(lPower)
        rPower = int(rPower)
        if reversing:
            lPower, rPower = -rPower, -lPower
        print(lPower, rPower)
        self.commands.append(f"{lPower:+04}{rPower:+04}")
        self.lastCommandTime = time.time()


    def setBeacons(self, beacons):
        """Update the internal location of the beacons if a simple heuristic favours the new list"""
        if len(beacons) >= len(self.beacons) and len(beacons) <= 3:
            self.beacons = beacons



