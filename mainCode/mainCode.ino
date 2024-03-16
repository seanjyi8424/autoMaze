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
 */

#include <Servo.h>

// *** SET THESE VARIABLES FIRST BEFORE UPLOADING TO VEHICLE ***
#define DRIVE_SELECT    1    // 1 = Main Drive Function, 2 = Lane Keep Drive Only, 3 = Ultrasonic Obstacle Avoidance
#define VEHICLE_SELECT  2    // Vehicle 1 = 1; Vehicle 2 = 2;
#define VEHICLE_SPEED   95   // Speed of the vehicle
#define TURN_SPEED      85   // Speed of the vehicle when turning
#define START_POS_X     2    // Starting Position in maze
#define START_POS_Y     0
#define START_DIRECTION EAST // Starting Direction in maze
// *** SET THESE VARIABLES FIRST BEFORE UPLOADING TO VEHICLE ***

// Motor Controller Pins
#define Lpwm_pin  5     // pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6     // pin of controlling speed---- ENB of motor driver board
const int pinLB = 2;    // pin of controlling turning---- IN1 of motor driver board left back
const int pinLF = 4;    // pin of controlling turning---- IN2 of motor driver board left forward
const int pinRB = 7;    // pin of controlling turning---- IN3 of motor driver board right back
const int pinRF = 8;    // pin of controlling turning---- IN4 of motor driver board right forward

// Ultrasonic Sensor Pins
Servo myservo;
const int Echo_Pin = A0; // ultrasonic module ECHO to A0
const int Trig_Pin = A1; // ultrasonic module TRIG to A1

// Obstacle Avoidance Sensor Pins
const int IR_L = 12; // pins
const int IR_R = 11;

#define MAX_PATH_LENGTH 50
#define GRID_WIDTH      4
#define GRID_LENGTH     6

const int UNEXPLORED = 0;

int exploration[GRID_WIDTH][GRID_LENGTH] = {0};

bool pathFound = false;
int stepCounter = 0;
int currPos[2] = {START_POS_X, START_POS_Y}; // Current position
int path[MAX_PATH_LENGTH][2] = {{START_POS_X, START_POS_Y}};

enum Direction {NORTH, SOUTH, EAST, WEST};
Direction currentDir = START_DIRECTION;

void turnRight() {
  switch (currentDir) {
    case NORTH: currentDir = EAST; break;
    case EAST: currentDir = SOUTH; break;
    case SOUTH: currentDir = WEST; break;
    case WEST: currentDir = NORTH; break;
  }
}

void turnLeft() {
  switch (currentDir) {
    case NORTH: currentDir = WEST; break;
    case WEST: currentDir = SOUTH; break;
    case SOUTH: currentDir = EAST; break;
    case EAST: currentDir = NORTH; break;
  }
}

// ----- Sensor Functions -----
// Bluetooth Module
void recvPath() {
  if (Serial.available() > 0) {
    // TODO: Implement receiving path
  }
}

void sendPath() {
  // TODO: Implement sending path
}

// Ultrasonic Sensor
#define USS_DETECT_DISTANCE 18 // Minimum distance to detect a wall/obstacle
volatile int Front_Distance;
volatile int Left_Distance;
volatile int Right_Distance;
bool USS_clearFront = false;
bool USS_clearLeft = false;
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

void USS_Main() {
  USS_clearFront = true;
  USS_clearLeft = true;
  USS_clearRight = true;

  Front_Distance = checkdistance();
  USS_CheckFront();

  if ((Front_Distance < USS_DETECT_DISTANCE) && (Front_Distance > 0)) {
    stopp();
    delay(150);
    USS_CheckFront();
    delay(150);
    USS_CheckLeft();
    delay(150);
    USS_CheckRight();
    delay(150);
    USS_ResetPos();

    USS_clearFront = (Front_Distance >= USS_DETECT_DISTANCE);
    USS_clearLeft = (Left_Distance >= USS_DETECT_DISTANCE);
    USS_clearRight = (Right_Distance >= USS_DETECT_DISTANCE);
  } else {
    USS_clearFront = true;
  }
}

// --- DRIVE MODES ---
#define TURN_DELAY 250 // Adjust value as needed to achieve 90-degree turns (milliseconds)

void Drive_Main() {
  USS_Main();

  if (USS_clearFront && USS_clearLeft && USS_clearRight) {
    if (!IR_L && !IR_R) {
      go_forward(VEHICLE_SPEED);
    } else if (IR_L && !IR_R) {
      while (!IR_R) {
        rotate_left(TURN_SPEED);
        IR_R = digitalRead(IR_R);
      }
      stopp();
      delay(500);
      go_forward(VEHICLE_SPEED);
      while (IR_L || IR_R) {
        IR_L = digitalRead(IR_L);
        IR_R = digitalRead(IR_R);
      }
      stopp();
      delay(500);
    } else if (!IR_L && IR_R) {
      while (!IR_L) {
        rotate_right(TURN_SPEED);
        IR_L = digitalRead(IR_L);
      }
      stopp();
      delay(500);
      go_forward(VEHICLE_SPEED);
      while (IR_L || IR_R) {
        IR_L = digitalRead(IR_L);
        IR_R = digitalRead(IR_R);
      }
      stopp();
      delay(500);
    } else if (IR_L && IR_R) {
      stopp();
      delay(500);
      go_forward(VEHICLE_SPEED);
      while (IR_L || IR_R) {
        IR_L = digitalRead(IR_L);
        IR_R = digitalRead(IR_R);
      }
      stopp();
      delay(500);
    }
  } else if (!USS_clearFront && !USS_clearLeft && USS_clearRight) {
    rotate_right(TURN_SPEED);
    delay(TURN_DELAY);
  } else if (!USS_clearFront && USS_clearLeft && USS_clearRight) {
    if (Left_Distance > Right_Distance) {
      rotate_left(TURN_SPEED);
    } else {
      rotate_right(TURN_SPEED);
    }
    delay(TURN_DELAY);
  } else if (!USS_clearFront && !USS_clearLeft && !USS_clearRight) {
    rotate_left(TURN_SPEED);
    delay(TURN_DELAY * 2);
  } else {
    stopp();
  }
}

void Ultrasonic_obstacle_avoidance() {
  Front_Distance = checkdistance();

  if ((Front_Distance < 10) && (Front_Distance > 0)) {
    stopp();
    delay(100);
    myservo.write(180);
    delay(500);
    Left_Distance = checkdistance();
    delay(100);
    myservo.write(0);
    delay(500);
    Right_Distance = checkdistance();
    delay(100);

    if (Left_Distance > Right_Distance) {
      rotate_left(TURN_SPEED);
      myservo.write(90);
      delay(300);
    } else {
      rotate_right(TURN_SPEED);
      myservo.write(90);
      delay(300);
    }
  } else {
    go_forward(VEHICLE_SPEED);
  }
}

// --- Motor Control Functions ---
void go_forward(unsigned char speed_val) {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void go_backward(unsigned char speed_val) {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
}

void rotate_left(unsigned char speed_val) {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, LOW);
  digitalWrite(pinLB, LOW);
  digitalWrite(pinLF, HIGH);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
  turnLeft();
}

void rotate_right(unsigned char speed_val) {
  digitalWrite(pinRB, LOW);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, LOW);
  analogWrite(Lpwm_pin, speed_val);
  analogWrite(Rpwm_pin, speed_val);
  turnRight();
}

void turnUntil(unsigned char speed_val) {
  while (currentDir != SOUTH) {
    Serial.println("Current Direction: " + String(currentDir));
    rotate_left(speed_val);
    delay(TURN_DELAY);
    stopp();
    delay(1000);
  }
  Serial.println("Stopped!");
  stopp();
}

void stopp() {
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinRF, HIGH);
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinLF, HIGH);
}

void setup() {
  myservo.attach(A2);
  Serial.begin(115200);
  USS_ResetPos();

  pinMode(Echo_Pin, INPUT);
  pinMode(Trig_Pin, OUTPUT);
  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);

  // Set all indexes as unexplored
  for (int i = 0; i < GRID_WIDTH; i++) {
    for (int j = 0; j < GRID_LENGTH; j++) {
      exploration[i][j] = UNEXPLORED;
    }
  }
}

void loop() {
  switch (DRIVE_SELECT) {
    case 1: Drive_Main(); break;
    case 2: Ultrasonic_obstacle_avoidance(); break;
    case 3: turnUntil(VEHICLE_SPEED); break;
    default:
      stopp();
      Serial.println("Error: Drive Mode not selected.");
      break;
  }
}