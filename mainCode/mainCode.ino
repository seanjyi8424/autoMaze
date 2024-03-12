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
#define DRIVE_SELECT   2    // 1 = Main Drive Function, 
                            // 2 = Lane Keep Drive Only, 
                            // 3 = Ultrasonic Obstacle Avoidance
#define VEHICLE_SELECT 1    // Vehicle 1 = 1; 
                            // Vehicle 2 = 2; 
#define START_POS_X    0    // Starting Position in maze
#define START_POS_Y    0
// *** SET THESE VARIABLES FIRST BEFORE UPLOADING TO VEHICLE ***

// Motor Controller Pins
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
const int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board left back
const int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board left forward
const int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board right back
const int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board right forward

// Ultrasonic Sensor Pins
Servo myservo;
const int Echo_Pin=A0;  // ultrasonic module   ECHO to A0
const int Trig_Pin=A1;  // ultrasonic module  TRIG to A1

// Obstacle Avoidance Sensor Pins
const int laneKeep_L= 12; // pins
const int laneKeep_R = 11;

// Coordinate Pathing Variables *** IMPORTANT TODO ***
/* Needed variables:
    - path (array of coordinates)
    - currentPos (current position)
    - nextPos (next position)
    - vehicleSelect (vehicle 1 or 2)
    - pathFound (boolean)
*/
#define MAX_PATH_LENGTH 10
#define MAX_GRID_LENGTH 10
#define MAX_GRID_WIDTH  10
#define GRID_WIDTH      4
#define GRID_LENGTH     6

const int UNEXPLORED = 0;
const int OPEN = 1;
const int WALL = 2;
const int VISITED = 3;

int grid[GRID_LENGTH][GRID_WIDTH] = {
  (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5),
  (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5),
  (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5),
  (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5)
  }; // Grid of coordinates (4x6) excluding the walls


int vehicleSelect = VEHICLE_SELECT;
bool pathFound = false;
int stepCounter = 0;
int currPos[2] = {START_POS_X, START_POS_Y}; // Current position
int path[2][MAX_PATH_LENGTH] = {(0, 0)}; // Path to follow

// Vehicle 2 Variables (the secondary vehicle)
int currPos2[2] = {0, 0}; // Current position
int path2[2][MAX_PATH_LENGTH] = {{0, 0}}; // Path to follow


// typedef Vector <int> currentPos; // Current position
// typedef Vector <Vector <int>> path; // Path to follow
// typedef Vector <Vector <int>> grid; // Grid of coordinates (4x6) excluding the walls

enum Direction {NORTH, SOUTH, EAST, WEST};
Direction currentDir = NORTH;

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

char command = 'S';
bool BT_sendCurrPos = false;
bool BT_sendPath    = false;
void sendCommunication() {
  if(BT_sendCurrPos == true) {
    Serial.write('C');
    sendPos();
  }
  if(BT_sendPath == true) {
    Serial.write('P');
    sendPath();
  }
  if(BT_sendCurrPos == false && BT_sendPath == false) {
    Serial.write('S');
  }
  else {
    Serial.write('E');
  }
}
void recvCommunication() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    switch(command) {
      case 'C':
        BT_sendCurrPos = true;
        break;
      case 'P':
        BT_sendPath = true;
        break;
      case 'S':
        BT_sendCurrPos = false;
        BT_sendPath = false;
        break;
      default:
        BT_sendCurrPos = false;
        BT_sendPath = false;
        break;
    }
  }

}
void recvPos() {
  if(Serial.available() > 0) {
    currPos2[0] = Serial.read();
    currPos2[1] = Serial.read();
  }
}
void sendPos() {
  Serial.write(currPos[0]);
  Serial.write(currPos[1]);
}


void recvPath() {
  if(Serial.available() > 0) {
    // path = Serial.read();

  }
}
void sendPath() {
  if(pathFound == true) {
    
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
// bool lineDetected_F = false;

void laneKeep() { 
  // TODO: Implement Lane Keep function using the two obstacle avoidance sensors
  // Read the state of the the input pin:
  // lineDetected_L = 0; // 0 = no line, 1 = line detected
  // lineDetected_R = 0;

  currState_L = digitalRead(laneKeep_L);
  currState_R = digitalRead(laneKeep_R);

  if (lastState_L == HIGH && currState_L == LOW) { 
    Serial.println("Line not detected on the left");
    lineDetected_L = 0; 
  } 
  else if (lastState_L == LOW && currState_L == HIGH) { 
    Serial.println("Line detected on the left");
    lineDetected_L = 1; 
    // If the line is detected on the left, then the vehicle should turn right
  }
  
  if (lastState_R == HIGH && currState_R == LOW) {
    Serial.println("Line not detected on the right");
    lineDetected_R = 0;
  }
  else if (lastState_R == LOW && currState_R == HIGH) {
    Serial.println("Line detected on the right");
    lineDetected_R = 1;
    // If the line is detected on the right, then the vehicle should turn left

  }

  // if ((currState_L == HIGH && currState_R == HIGH) &&
  //      lineDetected_L == lineDetected_R) {
  //     }

  lastState_L = currState_L;
  lastState_R = currState_R;
}

// Ultrasonic Sensor
#define USS_DETECT_DISTANCE 15 // Minimum distance to detect a wall/obstacle
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
  delay(400);
}
void USS_CheckLeft() {
  myservo.write(180);
  delay(400);
  Left_Distance = checkdistance();
  delay(600);
  Serial.print("Left_Distance:");
}
void USS_CheckRight() {
  myservo.write(0);
  delay(400);
  Right_Distance = checkdistance();
  delay(600);
  Serial.print("Right_Distance:");
  Serial.println(Right_Distance);
}
void USS_CheckFront() {
  myservo.write(90);
  Front_Distance = checkdistance();
  delay(600);
  Serial.print("Front_Distance:");
  Serial.println(Front_Distance);
}

void USS_Main() { // Original obstacle avoidance function, uses only the ultrasonic sensor
  USS_clearFront = true;
  USS_clearLeft  = true;
  USS_clearRight = true;

  Front_Distance = checkdistance(); // obtain the value detected by ultrasonic sensor 
  USS_CheckFront();
  if((Front_Distance < USS_DETECT_DISTANCE) && (Front_Distance > 0)) { 
      // if the distance is greater than 0 and less than USS_DETECT_DISTANCE  
    USS_clearFront = false;

    stopp(); //stop
    delay(150);
    USS_CheckFront();
    delay(600);
    USS_CheckLeft();
    delay(600);
    USS_CheckRight();
    delay(600);
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

void Drive_Main() {
  USS_clearFront = true;
  USS_clearLeft  = true;
  USS_clearRight = true;

  lineDetected_L = false;
  lineDetected_R = false;


  USS_Main();
  if (USS_clearFront == true && USS_clearLeft == true && USS_clearRight == true) {
    // Ultrasonic sensor clear
    laneKeep();
    if (lineDetected_L == 0 && lineDetected_R == 0) { // No line detected
      go_forward(100);
    }
    else if (lineDetected_L == 0 && lineDetected_R == 1) { // Line detected on the right
      laneKeep_left(100);
    }
    else if (lineDetected_L == 1 && lineDetected_R == 0) { // Line detected on the left
      laneKeep_right(100);
    }
    else if (lineDetected_L == 1 && lineDetected_R == 1) { // Line in front
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
    }
    else {
      stopp();
    }
  }
  else if (USS_clearFront == false &&
           USS_clearLeft  == true  && USS_clearRight == false) { // Left is clear
    rotate_left(150);
    delay(300);
  }
  else if (USS_clearFront == false && 
           USS_clearLeft  == false && USS_clearRight == true) {  // Right is clear
    rotate_right(150);
    delay(300);
  }
  else if (USS_clearFront == false && 
           USS_clearLeft  == true  && USS_clearRight == true) { // Left and Right are clear
    if (Left_Distance > Right_Distance) {
      rotate_left(150);
    }
    else {
      rotate_right(150);
    }
    delay(300);
  }
  else if (USS_clearFront == true  &&
           USS_clearLeft  == false && USS_clearRight == false) { // Front is clear
    go_forward(100);
  }
  else if (USS_clearFront == true && 
           USS_clearLeft  == true && USS_clearRight == true) { // All are clear
    go_forward(100);
  }
  else if (USS_clearFront == false &&
           USS_clearLeft  == false && USS_clearRight == false) { // All are blocked
    rotate_left(150); // replace with a 180 turn
    delay(600);
  }
  else {
    stopp();
  }
}

void laneKeep_Drive() {  // Motor Drive function using the lane keep function only
  laneKeep();
  if (lineDetected_L == 0 && lineDetected_R == 0) {
    go_forward(100);
  }
  else if (lineDetected_L == 0 && lineDetected_R == 1) {
    // rotate_left(100);
    laneKeep_left(100);
  }
  else if (lineDetected_L == 1 && lineDetected_R == 0) {
    // rotate_right(100);
    laneKeep_right(100);
  }
  else if (lineDetected_L == 1 && lineDetected_R == 1) {
    stopp();
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
      rotate_left(150);//turn left
      myservo.write(90);
      delay(300); 
    }
    else //if the right distance is greater than the left
    {
      rotate_right(150);// turn right
      myservo.write(90);
      delay(300); 
    }
  }
  else//otherwise
  {
    go_forward(100);//go forward
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
  digitalWrite(pinLB,HIGH); 
  digitalWrite(pinLF,HIGH);
  analogWrite(Lpwm_pin,speed_val / 2);
  analogWrite(Rpwm_pin,speed_val);
}
void laneKeep_right(unsigned char speed_val) { // Correct the vehicle to the right
  digitalWrite(pinRB,HIGH);                    // speed_val: 0~255
  digitalWrite(pinRF,HIGH);  
  digitalWrite(pinLB,HIGH); 
  digitalWrite(pinLF,LOW );
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val / 2);
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
  for (int i = 0; i < GRID_LENGTH; i++) {
    for (int j = 0; j < GRID_WIDTH; j++) {
      grid[i][j] = UNEXPLORED;
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
    default: 
      stopp();
      Serial.println("Error: Drive Mode not selected.");
      break;
    }
}
