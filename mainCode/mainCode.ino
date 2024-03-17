/* CS179J Autonomous Vehicle Maze Project
 * Group: ERASER
 * Members: Eric Choi, Artip Nakchinda, Sean Yi
 * 
 * filename: mainCode.ino
   * This file is the main code for the autonomous vehicle maze project.
   * It contains code consisting of the following:
   * - Motor Control Functions
   * - Ultrasonic Sensor Functions for detecting obstacles
   * - Obstacle Sensors for Lane Keep Functions
   * - Bluetooth Module Functions for sending/receiving coordinates
   * - Coordinate Pathing Functions
 *
 */
// Code inspired by original kit's code, 
// Modify to add obstacle avoidance sensor and bluetooth module
// also adding pathing functions
// May also modify for better organization (i.e. Convert to use state machine)

// Libraries
#include <Servo.h>
// #include <Vector.h>

// *** SET THESE VARIABLES FIRST BEFORE UPLOADING TO VEHICLE ***
#define DRIVE_SELECT    1    // 1 = Main Drive Function, 
                             // 2 = Lane Keep Drive Only, 
                             // 3 = Ultrasonic Obstacle Avoidance
#define VEHICLE_SELECT  2    // Vehicle 1 = 1; 
                            // Vehicle 2 = 2; 
#define VEHICLE_SPEED   90  // Speed of the vehicle
#define TURN_SPEED      85  // Speed of the vehicle when turning
#define START_POS_X     1    // Starting Position in maze
#define START_POS_Y     0
// Vehicle 1: 0,0
// Vehicle 2: 3,0
#define START_DIRECTION EAST // Starting Direction in maze
// *** SET THESE VARIABLES FIRST BEFORE UPLOADING TO VEHICLE ***

// Motor Controller Pins
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
const int pinLB = 2;             //pin of controlling turning---- IN1 of motor driver board left back
const int pinLF = 4;             //pin of controlling turning---- IN2 of motor driver board left forward
const int pinRB = 7;            //pin of controlling turning---- IN3 of motor driver board right back
const int pinRF = 8;            //pin of controlling turning---- IN4 of motor driver board right forward

// Ultrasonic Sensor Pins
Servo myservo;
const int Echo_Pin=A0;  // ultrasonic module   ECHO to A0
const int Trig_Pin=A1;  // ultrasonic module  TRIG to A1

// Obstacle Avoidance Sensor Pins
const int laneKeep_L = 12; // pins
const int laneKeep_R = 11;

// Coordinate Pathing Variables *** IMPORTANT TODO ***
/* Needed variables:
    - path (array of coordinates)
    - currentPos (current position)
    - nextPos (next position)
    - vehicleSelect (vehicle 1 or 2)
    - pathFound (boolean)
*/
#define MAX_PATH_LENGTH 50
#define MAX_GRID_LENGTH 30
#define MAX_GRID_WIDTH  30
#define GRID_WIDTH      4
#define GRID_LENGTH     6

const int UNEXPLORED = 0;
const int OPEN = 1;
const int WALL = 2;
const int VISITED = 3;

// int grid[GRID_WIDTH][GRID_LENGTH] = { 
//   (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5),
//   (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5),
//   (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5),
//   (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5)
//   }; // Grid of coordinates (4x6)

int exploration[GRID_WIDTH][GRID_LENGTH] = {
  (0, 0, 0, 0, 0, 0), // x, y -- -- >
  (0, 0, 0, 0, 0, 0), // |
  (0, 0, 0, 0, 0, 0), // |
  (0, 0, 0, 0, 0, 0), // v
  (0, 0, 0, 0, 0, 0)
  }; // exploration grid, 0 = unexplored, 1 = open, 2 = wall, 3 = visited

int vehicleSelect = VEHICLE_SELECT; // not sure if needed...

bool pathFound = false;
int stepCounter = 0;
int currPos[2] = {START_POS_X, START_POS_Y}; // Current position
int path[MAX_PATH_LENGTH][2] = {(START_POS_X, START_POS_Y) }; // Path to follow

// Vehicle 2 Variables (the secondary vehicle)
int currPos2[2] = {0, 0}; // Current position (not sure if needed)*
int path2[MAX_PATH_LENGTH][2] = {(0, 0)}; // Path to follow


// typedef Vector <int> currentPos; // Current position
// typedef Vector <Vector <int>> path; // Path to follow
// typedef Vector <Vector <int>> grid; // Grid of coordinates (4x6) excluding the walls

enum Direction {NORTH, SOUTH, EAST, WEST};
Direction currentDir = START_DIRECTION; 

void turnRight(){
  switch(currentDir){
    case NORTH:
      currentDir = EAST;
      break;
    case EAST:
      currentDir = SOUTH;
      break;
    case SOUTH:
      currentDir = WEST;
      break;
    case WEST:
      currentDir = NORTH;
      break;
  }
}

void turnLeft(){
  switch(currentDir){
    case NORTH:
      currentDir = WEST;
      break;
    case WEST:
      currentDir = SOUTH;
      break;
    case SOUTH:
      currentDir = EAST;
      break;
    case EAST:
      currentDir = NORTH;
      break;
  }
}

// ----- Sensor Functions -----
// Bluetooth Module 
// 
// TODO: Implement communication functions
//  - sendCommunication()
//  - recvCommunication()
//  - sendPos()
//  - recvPos()
//  - sendPath() // important functons
//  - recvPath()

char command = 'S'; // default: no data to transmit
int BT_stepCounter = 0;

bool BT_sendPath = false;

void recvPath() {
  if(Serial.available() > 0) {
    stopp();
    BT_stepCounter = Serial.read();
    // path = Serial.read();
    for (int i = 0; i < BT_stepCounter || Serial.available() > 0; i++) {
      for (int j = 0; j < 2; j++) {
        path2[i][j] = Serial.read();
      }
    }
    // etc...; though there's probably a better way to do this.

  }
}
void sendPath() {
  stopp();
  Serial.write(static_cast<int>(stepCounter)); // send the number of steps in path

  for(int i = 0; i < stepCounter; i++) { // send the path
    for(int j = 0; j < 2; j++) {
      Serial.write(path[i][j]);
    }
  }
}


// Uses the two obstacle avoidance sensors to detect lines and keep vehicle straight
// Lane Keep/Obstacle Avoidance Sensor Variables
int lastState_L = HIGH;  // the previous state from the input pin
int currState_L;        // the current reading from the input pin

int lastState_R = HIGH;
int currState_R;

bool lineDetected_L = false;
bool lineDetected_R = false;
bool lineTimeDetected_L = false;
bool lineTimeDetected_R = false;
// bool lineDetected_F = false;


void laneKeep() { 
  // TODO: Implement Lane Keep function using the two obstacle avoidance sensors
  // Read the state of the the input pin:
  // lineDetected_L = 0; // 0 = no line, 1 = line detected
  // lineDetected_R = 0;

  currState_L = digitalRead(laneKeep_L);
  currState_R = digitalRead(laneKeep_R);

  if (currState_L == LOW) {// lastState_L == HIGH && currState_L == LOW) { 
    Serial.println("Line not detected on the left");
    lineDetected_L = false; 
  } 
  else if (currState_L == HIGH) { // lastState_L == LOW && currState_L == HIGH) { 
    Serial.println("Line detected on the left");
    lineDetected_L = true; 
    // If the line is detected on the left, then the vehicle should turn right
  }
  
  if (currState_R == LOW) { // lastState_R == HIGH && currState_R == LOW) {
    Serial.println("Line not detected on the right");
    lineDetected_R = false;
  }
  else if (currState_R == HIGH) { // lastState_R == LOW && currState_R == HIGH) {
    Serial.println("Line detected on the right");
    lineDetected_R = true;
    // If the line is detected on the right, then the vehicle should turn left

  }

  // if ((currState_L == HIGH && currState_R == HIGH) &&
  //      lineDetected_L == lineDetected_R) {
  //     }

  // lastState_L = currState_L;
  // lastState_R = currState_R;
}

void laneKeep_stop() {
  laneKeep();
  if(lineDetected_L==true && lineDetected_R==true) {
    stopp();
  }
}

// Ultrasonic Sensor
#define USS_DETECT_DISTANCE 18 // Minimum distance to detect a wall/obstacle
volatile int Front_Distance;
volatile int Left_Distance;
volatile int Right_Distance;
bool USS_clearFront = false;
bool USS_clearLeft  = false;
bool USS_clearRight = false;

float checkdistance() {
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  float distance = pulseIn(Echo_Pin, HIGH) / 58.00;
  delay(10);
  return distance;
}

void USS_ResetPos() {
  myservo.write(90);
  delay(100);
}
void USS_CheckLeft() {
  myservo.write(180);
  delay(500);
  Left_Distance = checkdistance();
  Serial.print("Left_Distance:");
}
void USS_CheckRight() {
  myservo.write(0);
  delay(500);
  Right_Distance = checkdistance();
  Serial.print("Right_Distance:");
  Serial.println(Right_Distance);
}
void USS_CheckFront() {
  myservo.write(90);
  delay(500);
  Front_Distance = checkdistance();
  Serial.print("Front_Distance:");
  Serial.println(Front_Distance);
}

void USS_Main() { // Original obstacle avoidance function, uses only the ultrasonic sensor
  USS_clearFront = true;
  USS_clearLeft  = true;
  USS_clearRight = true;
  lineDetected_L = false;
  lineDetected_R = false;

  Front_Distance = checkdistance(); // obtain the value detected by ultrasonic sensor 
  USS_CheckFront();
  if((Front_Distance < USS_DETECT_DISTANCE) && (Front_Distance > 0)) { 
      // if the distance is greater than 0 and less than USS_DETECT_DISTANCE  
    USS_clearFront = false;

    stopp(); //stop
    delay(150);
    USS_CheckFront();
    delay(150);
    USS_CheckLeft();
    delay(150);
    USS_CheckRight();
    delay(150);
    USS_ResetPos();

    if (Front_Distance < USS_DETECT_DISTANCE) {
      USS_clearFront = false;
    }
    else {
      USS_clearFront = true;
    }

    if (Left_Distance < USS_DETECT_DISTANCE) {
      USS_clearLeft = false;
    }
    else {
      USS_clearLeft = true;
    }

    if (Right_Distance < USS_DETECT_DISTANCE) {
      USS_clearRight = false;
    }
    else {
      USS_clearRight = true;
    }
  }
  else { //otherwise
    USS_clearFront = true;
  }
}

// bool forward  = false; // dunno if we need this
// bool backward = false;
// bool left     = false;
// bool right    = false;
// bool stop     = true;

// --- DRIVE MODES ---
#define TURN_DELAY 250 // Adjust value as needed 
                       // to achieve 90-degree turns(milliseconds)

unsigned long sensorTriggerTimeL = 0;
unsigned long sensorTriggerTimeR = 0;
const unsigned long sensorDelay = 600; // 50 milliseconds

void Drive_Main() { // All good - Sean
  USS_clearFront = true;
  USS_clearLeft  = true;
  USS_clearRight = true;

  lineTimeDetected_L = digitalRead(laneKeep_L);
  lineTimeDetected_R = digitalRead(laneKeep_R);

  lineDetected_L = false;
  lineDetected_R = false;
  // Serial.println(millis());
  bool sensCheck = false;
  if (lineTimeDetected_L) {
    sensorTriggerTimeL = millis();
  }
  if (lineTimeDetected_R) {
    sensorTriggerTimeR = millis();
  }

   if ((lineTimeDetected_L) && ((millis() - sensorTriggerTimeR) <= sensorDelay)) {
    sensCheck = true;
   } else if((lineTimeDetected_R) && ((millis() - sensorTriggerTimeL) <= sensorDelay)) {
    sensCheck = true;
   } 
  USS_Main();
  if (USS_clearFront == true && USS_clearLeft == true && USS_clearRight == true) {
    // Ultrasonic sensor clear
    laneKeep();
    if (lineDetected_L == true && lineDetected_R == true && sensCheck == true) { // Line in front
      // while (lineDetected_L == 1 || lineDetected_R == 1) { 
      //   go_forward(VEHICLE_SPEED);
      //   laneKeep();
      // }
      go_forward(VEHICLE_SPEED);
      delay(500);
      stopp();
      switch (currentDir) {
        case NORTH:
          currPos[1]++; // Move up in the grid
          break;
        case EAST:
          currPos[0]++; // Move right in the grid
          break;
        case SOUTH:
          currPos[1]--; // Move down in the grid
          break;
        case WEST:
          currPos[0]--; // Move left in the grid
          break;
      } 
      path[0][stepCounter] = currPos[0];
      path[1][stepCounter] = currPos[1];
      stepCounter++;
      delay(1000);
      laneKeep();
    }
    else if (lineDetected_L == 0 && lineDetected_R == 1) { // Line detected on the right
      unsigned long startTime = millis();
      while (millis() - startTime <= 350) {
        go_backward(VEHICLE_SPEED);
        lineDetected_L = digitalRead(laneKeep_L);
        lineDetected_R = digitalRead(laneKeep_R);
        if (lineDetected_L == 1 && lineDetected_R == 1) {
          stopp();
          return;
        }
      }
      // startTime = millis();
      // while (millis() - startTime <= 350) {
      //   laneKeep_left(VEHICLE_SPEED);
      //   laneKeep();
      //   lineDetected_L = digitalRead(laneKeep_L);
      //   lineDetected_R = digitalRead(laneKeep_R);
      //   if (lineDetected_L == 1 && lineDetected_R == 1) {
      //     stopp();
      //     return;
      //   }
      // }
      laneKeep_left(VEHICLE_SPEED);
      laneKeep();
    } else if (lineDetected_L == 1 && lineDetected_R == 0) { // Line detected on the left
      unsigned long startTime = millis();
      while (millis() - startTime <= 350) {
        go_backward(VEHICLE_SPEED);
        lineDetected_L = digitalRead(laneKeep_L);
        lineDetected_R = digitalRead(laneKeep_R);
        if (lineDetected_L == 1 && lineDetected_R == 1) {
          stopp();
          return;
        }
      }
      // startTime = millis();
      // while (millis() - startTime <= 350) {
      //   laneKeep_right(VEHICLE_SPEED);
      //   laneKeep();
      //   lineDetected_L = digitalRead(laneKeep_L);
      //   lineDetected_R = digitalRead(laneKeep_R);
      //   if (lineDetected_L == 1 && lineDetected_R == 1) {
      //     stopp();
      //     return;
      //   }
      // }
      laneKeep_right(VEHICLE_SPEED);
      laneKeep();
    }
    else if (lineDetected_L == false && lineDetected_R == false) { // No line detected
      go_forward(VEHICLE_SPEED);
      laneKeep();
    }
    else {
      stopp(); // If something weird happens...
      laneKeep();
    }
  }
  else if (USS_clearFront == false &&
           USS_clearLeft  == true  && USS_clearRight == false) { // Left is clear
    rotate_left(TURN_SPEED);
    delay(TURN_DELAY);
  }
  else if (USS_clearFront == false && 
           USS_clearLeft  == false && USS_clearRight == true) {  // Right is clear
    rotate_right(TURN_SPEED);
    delay(TURN_DELAY);
  }
  else if (USS_clearFront == false && 
           USS_clearLeft  == true  && USS_clearRight == true) { // Left and Right are clear
    if (Left_Distance > Right_Distance) {
      rotate_left(TURN_SPEED);
    }
    else {
      rotate_right(TURN_SPEED);
    }
    delay(TURN_DELAY);
  }
  // else if (USS_clearFront == true  &&
  //          USS_clearLeft  == false && USS_clearRight == false) { // Front is clear
  //   laneKeep();
  //   go_forward(VEHICLE_SPEED);
  // }
  // else if (USS_clearFront == true && 
  //          USS_clearLeft  == true && USS_clearRight == true) { // All are clear
  //   laneKeep();
  //   go_forward(VEHICLE_SPEED);
  // }
  else if (USS_clearFront == false &&
           USS_clearLeft  == false && USS_clearRight == false) { // All are blocked
    laneKeep();
    rotate_left(TURN_SPEED); // replace with a 180 turn
    delay(TURN_DELAY*2);
  }
  else {
    stopp(); // If something weird happens...
  }
}

void laneKeep_Drive() {  // Motor Drive function using the lane keep function only
  laneKeep();
  if (lineDetected_L == false && lineDetected_R == false) {
    go_forward(VEHICLE_SPEED);
  }
  else if (lineDetected_L == false && lineDetected_R == true) {
    // rotate_left(VEHICLE_SPEED);
    laneKeep_left(VEHICLE_SPEED);
  }
  else if (lineDetected_L == true && lineDetected_R == false) {
    // rotate_right(VEHICLE_SPEED);
    laneKeep_right(VEHICLE_SPEED);
  }
  else if (lineDetected_L == true && lineDetected_R == true) {
    go_forward(VEHICLE_SPEED);
  }
  else { // If something weird happens...
    stopp();
    Serial.println("Error: Lane Keep Drive");
  }
}

void Ultrasonic_obstacle_avoidance() // Original obstacle avoidance function
{
  Front_Distance=checkdistance(); //obtain the value detected by ultrasonic sensor 
  if((Front_Distance < 10)&&(Front_Distance > 0))//if the distance is greater than 0 and less than 20  
{
    stopp();//stop
    delay(100);
    myservo.write(180);
    delay(500);
    Left_Distance=checkdistance();//measure the distance
    delay(100);
    myservo.write(0);
    delay(500);
    Right_Distance=checkdistance();//measure the distance
    delay(100);
if(Left_Distance > Right_Distance)//if distance a1 is greater than a2
    {
      rotate_left(TURN_SPEED);//turn left
      myservo.write(90);
      delay(300); 
    }
    else //if the right distance is greater than the left
    {
      rotate_right(TURN_SPEED);// turn right
      myservo.write(90);
      delay(300); 
    }
  }
  else//otherwise
  {
    go_forward(VEHICLE_SPEED);//go forward
  }
}
// --- DRIVE MODES ---


// --- Motor Control Functions ---
/* Notes:
 *  - speed_val: 0~255
 *  - HIGH = motor direction disabled
 *  - LOW  = motor direction enabled
 *  - There are 4 motor directions
 *    - pinRB: Right Back
 *    - pinRF: Right Forward
 *    - pinLB: Left Back
 *    - pinLF: Left Forward
 */
void go_forward(unsigned char speed_val) {   // speed_val：0~255
  digitalWrite(pinRB,HIGH); 
  digitalWrite(pinRF,LOW );
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW );
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val);
  
}
void go_backward(unsigned char speed_val) {   // speed_val：0~255
  digitalWrite(pinRB,LOW );  
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW );  
  digitalWrite(pinLF,HIGH);
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val);
}
void rotate_left(unsigned char speed_val) {      // speed_val：0~255
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW );  
  digitalWrite(pinLB,LOW ); 
  digitalWrite(pinLF,HIGH);
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val);
  turnLeft();
}
void rotate_right(unsigned char speed_val) {   // speed_val：0~255
  digitalWrite(pinRB,LOW );  
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW );  
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val);
  turnRight();
}
void laneKeep_left(unsigned char speed_val) { // Correct the vehicle to the left
  digitalWrite(pinRB,HIGH);                   // speed_val: 0~255
  digitalWrite(pinRF,LOW );  
  digitalWrite(pinLB,LOW); 
  digitalWrite(pinLF,HIGH);
  analogWrite(Lpwm_pin,speed_val / 2); // left motor
  analogWrite(Rpwm_pin,speed_val);
}
void laneKeep_right(unsigned char speed_val) { // Correct the vehicle to the right
  digitalWrite(pinRB,LOW);                    // speed_val: 0~255
  digitalWrite(pinRF,HIGH);  
  digitalWrite(pinLB,HIGH); 
  digitalWrite(pinLF,LOW );
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val / 2);
}

void turnUntilNot(unsigned char speed_val, Direction dir) {
  while (currentDir == dir) {
    Serial.println("Current Direction: " + currentDir);
    rotate_left(speed_val);
    delay(TURN_DELAY);
    stopp();
    delay(1000);
  }
  Serial.println("Stopped!");

  stopp();
}

void stopp() {      //stop
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);
}
// --- Motor Control Functions ---



void setup(){
  myservo.attach(A2);
  Serial.begin(115200);
  Front_Distance = 0;
  Left_Distance = 0;
  Right_Distance = 0;
  USS_ResetPos();
  vehicleSelect = VEHICLE_SELECT;
  pinMode(Echo_Pin, INPUT); // UltraSonic Sensor
  pinMode(Trig_Pin, OUTPUT);

  pinMode(laneKeep_L, INPUT); // Obstacle Avoidance Sensor
  pinMode(laneKeep_R, INPUT);

  // Motor Controller
  pinMode(pinLB,OUTPUT); // pin 2
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin 6(PWM)  

  // Set all indexes as unexplored //
  for (int i = 0; i < GRID_WIDTH; i++) {
    for (int j = 0; j < GRID_LENGTH; j++) {
      exploration[i][j] = UNEXPLORED;
    }
  }
}

void loop(){
  switch(DRIVE_SELECT) {
    case 1: // Main Drive Function
      Drive_Main();
      break;
    case 2: // Lane Keep Drive Only
      laneKeep_Drive();
      break;
    case 3: // Ultrasonic Obstacle Avoidance
      Ultrasonic_obstacle_avoidance();
      break;
    case 4:
      turnUntilNot(VEHICLE_SPEED, currentDir);
      break;
    default: 
      stopp();
      Serial.println("Error: Drive Mode not selected.");
      break;
    }
}

// unsigned long sensorTriggerTimeL = 0;
// unsigned long sensorTriggerTimeR = 0;
// const unsigned long sensorDelay = 50; // 50 milliseconds

// void loop() {
//   // Assuming sensorLeft and sensorRight are the left and right sensors
//   bool sensorL = digitalRead(laneKeep_L);
//   bool sensorR = digitalRead(laneKeep_R);

//   if (sensorL) {
//     sensorTriggerTimeL = millis();
//   }

//   if (sensorR) {
//     sensorTriggerTimeR = millis();
//   }

//   if (sensorL && millis() - sensorTriggerTimeR <= sensorDelay) {
    
//   } else if (sensorR && millis() - sensorTriggerTimeL <= sensorDelay) {
    
//   } else {
//     // Only one sensor triggered, lane keep this
//   }
// }
