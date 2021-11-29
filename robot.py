import threading
import socket
import math
import time


class Robot:
    def __init__(self):
        self.commands = []
        self.beacons = [
            (0,0),
            (0,0),
            (0,0)
        ]
        self.returning = False
    
    def connect(self, IPAddr, IPPort, MACAddr, MACPort):
        #self.bts = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.ips = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        #self.bts.bind((MACAddr, MACPort))
        self.ips.bind((IPAddr, IPPort))

        #self.bts.listen()
        self.ips.listen()
        
        self.t = threading.Thread(target=self.IPDaemon, daemon=True)
        self.t.start()

        self.commands = []

        self.lastCommandTime = 0

    def IPDaemon(self):
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
        xa = (x1 + x2) / 2
        ya = (y1 + y2) / 2
        position = (xa, ya)
        print("computing motors")
        print(f"beacons: {self.beacons}")
        i = 0
        while i<len(self.beacons) and self.beacons[i][0]-self.beacons[i][1] > 0:
            print(self.beacons)
            print(self.beacons[i])
            i+=1
        if i == len(self.beacons):
            print("all beacons on other side")
            return

        if not self.returning:
            finalDestination = self.beacons[i]
            print(f"final desination: {finalDestination}")
            print(xa, ya)
            if ((xa-300)**2 + (ya-300)**2)**0.5 < 160 and abs(xa+ya-600)<40 and ya - xa > 180:
                instrumentalDestination = (200,400)
            elif xa - ya > 0:
                instrumentalDestination = (400,200)
            else:
                instrumentalDestination = finalDestination
            print(instrumentalDestination)
        else:
            finalDestination = (520, 80)
            if ((xa-300)**2 + (ya-300)**2)**0.5 < 160 and abs(xa+ya-600)<40 and ya - xa < -180:
                instrumentalDestination = (400,200)
            elif xa - ya < 0:
                instrumentalDestination = (200,400)
            else:
                instrumentalDestination = finalDestination
            print(instrumentalDestination)

        orientation = [y2 - y1, x2 - x1]
        print(f"orientation: {orientation}")
        direction = [ya - instrumentalDestination[1], xa - instrumentalDestination[0]]
        print(f"direction: {direction}")

        directionToFinal = [ya - finalDestination[1], xa - finalDestination[0]]

        distance = (directionToFinal[0]**2 + directionToFinal[1]**2)**0.5

        if distance < 95:
            print("begun approach")
            #self.commands.append("approach")
            self.lastCommandTime = time.time()
            self.beacons.pop(i)
            self.returning = True
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
        print(lPower, rPower)
        self.commands.append(f"{lPower:+04}{rPower:+04}")
        self.lastCommandTime = time.time()


    def setBeacons(self, beacons):
        if len(beacons) >= len(self.beacons) and len(beacons) <= 3:
            self.beacons = beacons



