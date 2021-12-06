#include <SPI.h>
#include <WiFiNINA.h>

#include <stdlib.h>
#include <string.h>
#include <avr/wdt.h>

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

int irINP_1 = 7;	// The pin used for the first IR input voltage
int irINP_2 = 8;	// The pin used for the second IR input voltage
int ledOUT_1 = 12;	// The pin used for the green LED output voltage
int ledOUT_2 = 13;	// The pin used for the red LED output voltage
int distINP = A0;	// The pin used for the distance sensor input reading
int movingLED = 10;	// The pin used for the amber LED output voltage



char ssid[] = "camSpot";        // network SSID
char pass[] = "whiteboardtiledcarpet";    // network password
int status = WL_IDLE_STATUS;     // the WiFi radio's status

WiFiClient client;

boolean alreadyConnected = false; // whether or not the client was connected previously

void turn(int rate, int clockwise, int pause, int offset) {
  
  // Subroutine to turn or move the robot 

  digitalWrite(movingLED, 1);	// Turns on the amber LED
  leftMotor->setSpeed(rate);	// Sets the motors at the desired speed. Offset used to remove any differences in motor power.
  rightMotor->setSpeed(rate+offset);
  if (clockwise == 1) {		// Turns clockwise
    leftMotor->run(BACKWARD);   // Motors installed wrong way around, backward is forward and vice versa
    rightMotor->run(FORWARD);
    delay(pause);
  } else {
    if (clockwise == 0) { 	// Turns anticlockwise
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    delay(pause);
    } else {
      if (clockwise == 2) { 	// Moves backwards
        leftMotor->run(FORWARD);
        rightMotor->run(FORWARD);
        delay(pause);
      } else {			// Moves forwards
        rightMotor->run(BACKWARD);
        leftMotor->run(BACKWARD);
        delay(pause);
      }
    }

  }
  digitalWrite(movingLED, 0); 	// Turns the amber LED off
}

void locate() {
  
  // Points the robot to face the dummy

  int kickback = 108;		// The rate of turning back, should rotate the robot half of the original power
  int lowerBOUND = 260;		// The lower distance value for detecting a dummy
  int upperBOUND = 280;		// The upper distance value for detecting a dummy
  int inarow = 0;		// How many of a high reading has been detected in a row
  int count = 0;		// Variable to count how far the dummy has rotated
  int distVAL = getDistance();	// Reads the distance value from the distance sensor
  int amplitude = 0;		// Variable to calculate if the robot is moving clockwise or anticlockwise
  int linedUP = 0;		// Set to 1 when lined up
  if (distVAL <= upperBOUND) {
    
    //If it's not facing the dummy, finds the edge of the dummy
    
    while (linedUP == 0) {
      amplitude = amplitude + 1;
      count = 0;
      while (count <= 100 * amplitude and linedUP == 0) {	// While the robot has not turned a set distance or seen the dummy
        int distVAL = getDistance();
        if (amplitude % 2 == 0) {	// Turns clockwise, then anticlockwise, then clockwise with increasing oscillations
          turn(120, 0, 10, 0);
        } else {
          turn(120, 1, 10, 0);
        }
        count = count + 1;
        if (distVAL >= lowerBOUND) {	// If the dummy has been spotted
          inarow += 1;
          if (inarow >= 3) {		// If the dummy has been spotted three times in a row
            leftMotor->run(RELEASE);
            rightMotor->run(RELEASE);
            linedUP = 1;		// Stops robot
          }
        } else {
          if (inarow > 0) {		// Removes noise
            inarow -= 1;
          }
        }
      }
    }
  } else { 
    
    //If it's already facing the dummy, finds the edge of the dummy
    
    while (linedUP == 0) {		// Turns until the edge of the dummy is found
      int distVAL = getDistance();
      turn(120, 1, 10, 0);
      if (distVAL <= upperBOUND) {	// If the dummy has been lost, it's the edge of the dummy
        inarow += 1;
        if (inarow >= 2) {		// If the edge of the dummy has been found twice in a row, stops
          distVAL = getDistance();
          turn(120, 0, 250, 0);
          leftMotor->run(RELEASE);
          rightMotor->run(RELEASE);
          linedUP = 1;
        }
      } else {
        if (inarow > 0) {
          inarow -= 1;
        }
      }
      
    }
    amplitude = 0;	// Makes sure the counter rotate is in the correct direction
    
  }
  count = 0; 
  
  //Rotates until it's found the other side of the dummy
  
  int clockwise = amplitude % 2;	// Sets the counter rotate to the correct direction
  while (linedUP == 1) {		// Turns until the edge of the dummy is found
    turn(120, clockwise, 10, 0);
    count = count + 1;			// Counts for how many timesteps the dummy is in view
    int distVAL = getDistance();
    if (distVAL <= lowerBOUND) {	// Finds when the dummy leaves view
      linedUP = 0;
    }
  }
  
  //Rotates back to the centre of the dummy
  
  if (clockwise == 1) {			// Flips the rotation of the robot
    clockwise = 0;
  } else {
    clockwise = 1;
  }
  while (count >= 0) {
    turn(kickback, clockwise, 10, 0);	// Counter rotates at a slower rate to reach the centre of the detected object
    count = count - 1;
  }
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}



int identifyBeacons() {

  // Identifies the emitted LED signal and flashes the correct LED

  pinMode(ledOUT_1, OUTPUT);		// Resets pin outputs due to issues with definitions in scopes
  pinMode(ledOUT_2, OUTPUT);

  int irVAL_1 = digitalRead(7);

  int irVAL_2 = digitalRead(8);

  if (irVAL_1 == 0) {			// If the signal is 38KHz, 600us pulse, 10% duty cycle
    if (irVAL_2 == 0) {
      digitalWrite(ledOUT_1, 1);	// Sets red and green LEDs as on
      digitalWrite(ledOUT_2, 1);
    } else {				// If the signal is alternating pulse
      digitalWrite(ledOUT_1, 1);	// Sets green LED as on
      digitalWrite(ledOUT_2, 0);
    }

  } else {				// If the signal is 600us pulse, 10% duty cycle
    digitalWrite(ledOUT_1, 0);		// Sets red LED as on
    digitalWrite(ledOUT_2, 1);	
  }
  delay(5500);				// Waits 5 and a half seconds
  digitalWrite(ledOUT_1, 0);		// Turns LED off
  digitalWrite(ledOUT_2, 0);
  return 0;
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  Serial.println("Wifi test");

  //wdt_enable(WDTO_1S);

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  Serial.println("Wifi module found");

  // ensure the wifi firmware is up to date
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    //Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 3 seconds for connection:
    delay(3000);
  }

  
  Serial.print("You're connected to the network");

  // connect to the server socket
  while (!client.connect("192.168.137.1", 8081)) {
    Serial.println("trying connected to computer");
    delay(500);
  }

  Serial.println("connected to computer");
  
  // initialize motors
  leftMotor->setSpeed(100);
  rightMotor->setSpeed(100);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(10);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

  // read command from the server and discard
  char spd[100] = {0};
  int i = 0;
  while (client.available()) {
    spd[i] = client.read();
    i += 1;
  }
  
  Serial.println("finished setup");
}

void approach() {

  // Approaches the dummy until it is in grabbing distance

  int distVAL = getDistance();		// Gets distance
  int stopped = 0;			// Variable to show how many approaches have been carried out
  while (stopped == 0) {
    int distVAL = getDistance();
    
    turn(110, 3, 5, -5);		// Approaches the dummy
    if (distVAL > 450) {		// If close to the dummy, stops
      delay(100);
      stopped = 1;
    }
  }
  locate();				// Relocates the dummy
  while (stopped == 1) {
    distVAL = getDistance();
    
    turn(110, 3, 5, -5);		// Repeats the approach until grabbing distance
    if (distVAL > 680) {
      delay(100);
      stopped = 2;
    }
    
  }
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void loop() {
  char spd[100] = {0};  // string command
  char s1[30] = {0}; // string of speed 1
  char s2[30] = {0};  // string of speed 2
  int spd1;  // int of speed 1
  int spd2;  // int of speed 2
  int i = 0;
  int error;
  int distance;

  // read command from the server
  while (client.available()) {
    spd[i] = client.read();
    i += 1;
  }
  delay(50);

  Serial.println("spd command:");
  Serial.println(spd);

  if (strcmp(spd, "approach") == 0) {  // check for the keyword command 'approach'
    Serial.println("begun approach");

    delay(100);
    locate();

    delay(100);
    approach();

    identifyBeacons();

  } else if (strcmp(spd, "stop") == 0) {  // check for the keyword command 'stop'
    setMotors(0,0);
    while (true);
  }

  memcpy(s1, spd, 4*sizeof(char));
  memcpy(s2, &spd[4], 4*sizeof(char));
  // split the command into two values

  spd1 = atoi(s1);
  spd2 = atoi(s2);
  // convert string into int

  if (strlen(spd) != 0) {
    error = setMotors(spd1, spd2);
  }
  delay(100);
}

int setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  int leftDirection;
  int rightDirection;

  Serial.println("set motors");
  digitalWrite(movingLED, 1);  // enable the 555 flashing LED

  if (leftMotorSpeed >= 0) {
    leftDirection = FORWARD;
  } else {
    leftDirection = BACKWARD;
    leftMotorSpeed = - leftMotorSpeed;
  }
  if (rightMotorSpeed >= 0) {
    rightDirection = FORWARD;
  } else {
    rightDirection = BACKWARD;
    rightMotorSpeed = - rightMotorSpeed;
  }
  // calculate the direction and speed of each motor

  leftMotor->setSpeed(leftMotorSpeed);
  rightMotor->setSpeed(rightMotorSpeed);
  leftMotor->run(leftDirection);
  rightMotor->run(rightDirection);
  // set the direction and speed of each motor

  delay(100);
  digitalWrite(movingLED, 0);
  //disable the 555 flashing LED
}

int getDistance() {

  // Reads the output of the distance sensor and returns the value

  int distVAL = analogRead(A0);
  return distVAL;
}