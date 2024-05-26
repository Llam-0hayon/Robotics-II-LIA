/*
I2C Slave Control For ROBOTICS II LIA


This is the slave code for a system designed for the Elegoo Smart Car V4.0.
The purpose of this code is to use the data received by the master and an IR remote to determine how to control the vehicle.
If the Slave receives the numerical value 1 from the master arduino it means the master has scanned the RFID a robot car in front of it and had granted the car access.
When it Receives the numerical value for 1 the slave will execute a program that will have the car follow the closest object in this being the car in front of it.
Depending on the code the IR Receiver Receives the car will execute different functions relating to its movement.
It's important to note when the car receives the hexadecimal code to stop it resets the value of the code it receives from the master to 0.
This is done so that the car can be stopped from following and the user can regain manual control until the car scans another RFID.


Below are more details on how the Car Follows the object in front of it:
  The purpose of the follow code code is to make a elegoo smart car V4.0 follow a object or person within the range of the cars ultra sonic sensor.
  The Code mainly uses a series of if  statements to do the logic for deciding which direction to go in when to move in tht a direction and when to stop.
  The movement is done mostly by the creation and calling of functions for each direction that car moves in.
  AS well as a function for finding the shortest distance.
  In summary This mostly requires:
  -if else
  -Servo.h Library
  -digitalWrite
  -analogWrite


  The main way the following code functions is by having the servo on which the ultrasonic sensor is attached to move forward the left then right.
  A measurement is taken and saved at each direction the servo is pointed in.
  Then a function is called that finds the shortest distance out of the three then moves then rotates the car in the shortest direction.
  The car will be aligned to where the ultrasonic sensor was when it had taken that measurement


The Way the IR control function is detailed Below:
  By retrieving and storing any hex value the IR receiver receives.
  Each button on the ir remote has a certain hex value.
  If the received value is = the value of the forward button the car will go forward.
  If the received value is = the value of the Backward button the car will go backward.
  If the received value is = the value of the Right button the car will go Right.
  If the received value is = the value of the Left button the cra will go Left.
 
  This is done by using:
  -IRremote.h libraries
  -analogWrite
  -digitalWrite
  -if & else if


PIN CONFIGS:
Pins for IR receiver:
- Vcc pin    = pin 1 = 5V+
- GND pin    = pin 2 = GND
- signal pin = pin 1 = pin 9 on arduino


The pins for the ultrasonic sensor:
- Vcc pin  =  pin 1 = 5V+
- Trig pin =  pin 2 = pin 13 on arduino
- Echo pin =  pin 3 = pin 12 on arduino
- GND pin  =  pin 4 = GND


Pins for servo motor(P10)
- GND pin    = pin 1 = GND
- Vcc pin    = pin 2 = 5V+
- Signal pin = pin 3 = pin 10 on a arduino


Pins for dc motors
- AIN1(Direction)             = pin 10 on TB6612FNG = pin 8 on arduino
- PWMA(Speed/amount of power) = pin 9 on TB6612FNG  = pin 5 on arduino
- BIN1(Direction)             = pin 6 on TB6612FNG  = pin 7 on arduino
- PWMB(Speed/amount of power) = pin 7 on TB6612FNG  = pin 6 on arduino
- STBY                        = pin 19 on TB6612FNG = pin 3 on arduino


I2C Pin Configuration
Master:                    Slave:
- SDA                      - SDA
- SCL                      - SCL


Go to: https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial
For more info on components pin configuration/Data sheets as well as info on the follow function and other functions this car
can perform.
 */


//_____IR REMOTE INITIALIZATION & VARIABLES_____________________________________
#include <IRremote.h>
// Define the IR receiver pin
const int IR_PIN = 9;
// Define the IR receiver object
IRrecv irrecv(IR_PIN);

//_____SERVO INITIALIZATION & VARIABLES_________________________________________
// Including the Servo library for controlling the servo motor.
#include <Servo.h>
// Creating a Servo object named myservo to control the servo motor.
Servo myservo; 
// Defining pin numbers for ultrasonic sensor trigger, echo, and servo signal.

//_____ULTRA SONIC INITIALIZATION & VARIABLES___________________________________
#define trigPin 13                                                                  // Trigger pin
#define echoPin 12                                                                  // Echo pin
#define servoSig  10                                                                // Signal pin 

//_____DC MOTORS INITIALIZATION, VARIABLES & Functions__________________________
// Defining pin numbers for motor control and standby.
int PWMA = 5;                                               // speed/power for motor A.
int AIN1 = 7;                                               // Direction motor B.
int BIN1 = 8;                                               // Direction motor A.
int PWMB = 6;                                               // Speed/power for motor B.
int STBY = 3;                                               // Standby pin (allows power to motors).

// Defining speed values for different motor speeds.
int MINspeed = 50;                                          // Minimum speed before motors struggle.
int SPEED1 = 100;                                           // A speed value.
int SPEED2 = 150;                                           // A speed value.
int SPEED3 = 200;                                           // A speed value.
int MAXspeed = 255;                                         // Define the maximum motor speed.

// Define amount of milliseconds to go 180 degrees when going right
int oneEighty = 300; 

// Define the maximum distance for obstacle detection.
int maxDis = 18;
int minDis = 3;

// Funtion to make the car stop
void stop() {
  digitalWrite(STBY, 1);   // Give power to standby pin

  // Motor A
  pinMode(PWMA, OUTPUT);   // Set PWMA pin as OUTPUT
  analogWrite(PWMA, 0);    // Set the speed of motor A to zero
  pinMode(AIN1, OUTPUT);   // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);   // Set the direction of motor A (Set it HIGH to rotate clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);   // Set PWMB pin as OUTPUT
  analogWrite(PWMB, 0);    // Set the speed of motor B to zero
  pinMode(BIN1, OUTPUT);   // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);   // Set the direction of motor B (Set it HIGH to rotate clockwise)
}

// Function to make the car move forward
void forward(int speed) {
  digitalWrite(STBY, 1);       // Give power to standby pin

  // Motor A
  pinMode(PWMA, OUTPUT);       // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);    // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);       // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);       // Set the direction of motor A (Set it HIGH to rotate clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);       // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);    // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);       // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);       // Set the direction of motor B (Set it HIGH to rotate clockwise)
}

// Function to make the car move left
void left(int speed) {
  digitalWrite(STBY, 1);     // Give power to standby pin   

  // Motor A
  pinMode(PWMA, OUTPUT);     // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);  // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);     // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 1);     // Set the direction of motor A (Set it HIGH to rotate clockwise)    

  // Motor B
  pinMode(PWMB, OUTPUT);     // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);  // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);     // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 0);     // Set the direction of motor B (Set it LOW to rotate counter clockwise)
}

// Function to make the car move right
void right(int speed) {
  digitalWrite(STBY, 1);     // Give power to standby pin   

  // Motor A
  pinMode(PWMA, OUTPUT);      // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);   // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);      // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 0);      // Set the direction of motor A (Set it LOW to rotate clockwise)    

  // Motor B
  pinMode(PWMB, OUTPUT);        // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);     // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);        // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 1);        // Set the direction of motor B (Set it HIGH to rotate counter clockwise)
}

// Function to make the car move backward
void backward(int speed) {
  digitalWrite(STBY, 1);       // Give power to standby pin

  // Motor A
  pinMode(PWMA, OUTPUT);       // Set PWMA pin as OUTPUT
  analogWrite(PWMA, speed);    // Set the speed of motor A using PWM
  pinMode(AIN1, OUTPUT);       // Set AIN1 pin as OUTPUT
  digitalWrite(AIN1, 0);       // Set the direction of motor A (Set it LOW to rotate counter clockwise)

  // Motor B
  pinMode(PWMB, OUTPUT);       // Set PWMB pin as OUTPUT
  analogWrite(PWMB, speed);    // Set the speed of motor B using PWM
  pinMode(BIN1, OUTPUT);       // Set BIN1 pin as OUTPUT
  digitalWrite(BIN1, 0);       // Set the direction of motor B (Set it LOW to rotate counter clockwise)
}


//___SERVO FUNCTIONS______________________________________
// Function to make the servo move to the middle
void servoMiddle() { 
  myservo.write(80);                                    // Set the servo to 90 degrees
}                                  

// Function to make the servo move to the right
void servoRight() {
  myservo.write(0);                                     // Set the servo to 0 degrees
}

// Function to make the servo move to the left
void servoLeft() {
  myservo.write(175);                                   // Set the servo to 180 degrees
}

//_____ULTRA SONIC FUNCTIONS_______________________________
// Function to measure the distance using the ultrasonic sensor
float distanceRead() {
  digitalWrite(trigPin, LOW);                           // Set trigPin to LOW to ensure a clean pulse
  delayMicroseconds(2);                                 // Short delay to allow any lingering signals to settle
  digitalWrite(trigPin, HIGH);                          // Generate a 10-microsecond pulse by setting trigPin to HIGH
  delayMicroseconds(10);                                // Generate a 10-microsecond pulse by setting trigPin to HIGH
  digitalWrite(trigPin, LOW);                           // Reset trigPin to LOW to complete the pulse
  long duration = pulseIn(echoPin, HIGH);               // Measure the duration of the pulse received on echoPin
  float distance = (duration * 0.034 / 2);              // Calculate the distance using the speed of sound (0.0343 cm/microsecond)
  Serial.print("inside func = ");
  Serial.println(distance);
  return distance;                                      // Return the calculated distance
} 

//_____FUNCTION TO FIND SHORTEST DISTANCE___________________
float dF, dR, dL;  
void shortDisFind(float dL, float dF, float dR){
  if(dF >= dR) { 
    if(dR >= dL || 1 >= dL){
      left(80);
      delay(20);        
    } else if (1 >= dR || dR <= dL){
     right(80);   
     delay(20);    
    } else if(1 >= dL || dF >= dL){
      left(80);
      delay(20);  
    }
  }
}

//_______I2C INITIALIZATION & VARIABLES_____________________
// Include Arduino Wire library for I2C
#include <Wire.h>
// Define Slave I2C Address
#define SLAVE_ADDR 9
int rd;

void setup() {
//_______SERIAL MONITOR SETUP_______________________________
  Serial.begin(9600);
  Serial.println("I2C Slave");

//_______I2C SETUP__________________________________________
  // Initialize I2C communications as Slave
  Wire.begin(SLAVE_ADDR);
  // Function to run when data received from master
  Wire.onReceive(receiveEvent);
  // Setup Serial Monitor 
   
  myservo.attach(servoSig);                            // Attaches the servo motor to its signal pin
  Serial.begin(9600);                                  // Starts serial communication at 9600 baud rate
  pinMode(trigPin, OUTPUT);                            // Sets trigPin as an output
  pinMode(echoPin, INPUT);      

  irrecv.enableIRIn();
}

//______I2C RECEIVE FUNCTION________________________________
void receiveEvent() {
  // Read one character from the I2C
  rd = Wire.read();
  // Print value of incoming data
  Serial.println(rd);  

}

//______HOW CAR ISBEING CONTROLLED___________________________
void loop() {
  // Check if there are any available codes in the buffer
  if (irrecv.decode()) {
    // Print the HEX value of the button press
    unsigned long receivedCode = irrecv.decodedIRData.decodedRawData;
    Serial.println(receivedCode);

    // Define the codes as variables
    const unsigned long stopCode = 2072271525;        // ENTER button
    const unsigned long backwardCode = 2122406565;    // BACKWARD arrow button
    const unsigned long forwardCode = 2139118245;     // FORWARD arrow button
    const unsigned long leftCode = 2088983205;        // LEFT arrow button
    const unsigned long rightCode = 2105694885;       // RIGHT arrow button
    const unsigned long stopfollow = 4111096485;

    // Check if the received code matches the stop code
    if (receivedCode == stopCode) {
      stop(); // Stop the motors when the stop code is received
      rd = 0;
    }
    // Check if the received code matches the backward code
    else if (receivedCode == backwardCode) {
      backward(100); // Execute backward function with a speed of 100
    }
    // Check if the received code matches the forward code
    else if (receivedCode == forwardCode) {
      forward(100); // Move forward with a speed of 100
    }
    // Check if the received code matches the left code
    else if (receivedCode == leftCode) {
      left(100); // Turn left with a speed of 100
    }
    // Check if the received code matches the right code
    else if (receivedCode == rightCode) {
      right(100); // Turn right with a speed of 100
    }
    if (rd == 1) {
      // Move servo to middle position
      servoMiddle();
      delay(400);
      // Read distances
      float dF = distanceRead();
      Serial.print("dF=");
      Serial.println(dF);
      // Move servo to right position
      servoRight();
      delay(400);
      float dR = distanceRead();
      Serial.print("dR=");
      Serial.println(dR);
      // Move servo to left position
      servoLeft();
      delay(400);
      float dL = distanceRead();
      Serial.print("dL=");
      Serial.println(dL);
      // Find short distance and make turns accordingly
      shortDisFind(dL, dF, dR);
      // Move servo to middle position
      servoMiddle();
      delay(400);
      // Read distance again
      float dx = distanceRead();
      // Act based on distance
      if (minDis <= dx && dx <= maxDis) {
        forward(60);
        Serial.print("FOR=");
        Serial.println(dx);
      } else if (minDis >= dx) {
        stop();
        backward(60);
        delay(10);
      } else if (dx >= maxDis) {
        stop();
      }
    }
  }
  // Reset the IR receiver for the next signal
  irrecv.resume();
}
