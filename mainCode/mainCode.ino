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
#include <Vector.h>

// --- Constants and Variables ---
#define PERIOD 100

// Ultrasonic Sensor
Servo myservo;
const int Echo_Pin=A0;  // ultrasonic module   ECHO to A0
const int Trig_Pin=A1;  // ultrasonic module  TRIG to A1
volatile int Front_Distance;
volatile int Left_Distance;
volatile int Right_Distance;

// Motor Controller 
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
const int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
const int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
const int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
const int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board

// Coordinate Pathing Variables
bool pathFound = false;
// vector<int> startPos;
// vector<int> currPos;
// vector<int> endPos;
// vector<vector<int>> exitPath;
// int startPos[], currPos[], endPos[];
// int exitPath[][]; 

// Lane Keep/Obstacle Avoidance Sensor Variables
const int laneKeep_L= 12; // pins
const int laneKeep_R = 11;
int lastState_L = HIGH;  // the previous state from the input pin
int currState_L;        // the current reading from the input pin
int lastState_R = HIGH;
int currState_R;



// Bluetooth Module Variables
// Unnecessary for now, can just use Serial
// int bluetooth_rx = 9;
// int bluetooth_tx = 8;

// --- Constants and Variables (end) ---


// ----- User Added Functions -----
// Bluetooth Module
void bluetoothModule() {
    // TODO: Implement Bluetooth module to send/receive coordinates
}

// ----- Sensor Functions -----

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

void Detect_Left_and_Right__distance() {
  myservo.write(180);
  delay(400);
  Left_Distance = checkdistance();
  delay(600);
  Serial.print("Left_Distance:");
  Serial.println(Left_Distance);
  myservo.write(0);
  delay(400);
  Right_Distance = checkdistance();
  delay(600);
  Serial.print("Right_Distance:");
  Serial.println(Right_Distance);
  myservo.write(90);
}

// Uses the two obstacle avoidance sensors to detect lines and keep vehicle straight
int lineDetected_L = 0;
int lineDetected_R = 0;

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

  if ((currState_L == HIGH && currState_R == HIGH) 
    && lineDetected_L == lineDetected_R) {
      Serial.println("Line detected in front");
      // If the line is detected in front, then the vehicle should know
      // that it has crossed to the next coordinate

    }
}

void Ultrasonic_obstacle_avoidance() {
  Front_Distance=checkdistance(); //obtain the value detected by ultrasonic sensor 
  if((Front_Distance < 10)&&(Front_Distance > 0)) { //if the distance is greater than 0 and less than 20  
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
    if(Left_Distance > Right_Distance) { //if distance a1 is greater than a2
          rotate_left(150);//turn left
          myservo.write(90);
          delay(300); 
        } else { //if the right distance is greater than the left
            rotate_right(150);// turn right
            myservo.write(90);
            delay(300); 
        }
    }
    else { //otherwise
      go_forward(100);//go forward
    }
}

void Obstacle_Avoidance_Main() {
  Ultrasonic_obstacle_avoidance();
  // laneKeep();
}


// --- Motor Control Functions ---
void go_forward(unsigned char speed_val) {   // speed_val：0~255
  digitalWrite(pinRB,HIGH); 
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val);
  
}

void go_backward(unsigned char speed_val) {   // speed_val：0~255
  digitalWrite(pinRB,LOW);  
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);  
  digitalWrite(pinLF,HIGH);
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val);
}
    
void rotate_left(unsigned char speed_val) {      // speed_val：0~255
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW );  
  digitalWrite(pinLB,LOW); 
  digitalWrite(pinLF,HIGH);
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val);

}
void rotate_right(unsigned char speed_val) {   // speed_val：0~255
  digitalWrite(pinRB,LOW);  
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);  
  analogWrite(Lpwm_pin,speed_val);
  analogWrite(Rpwm_pin,speed_val);
     
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
  myservo.write(90);
  pinMode(Echo_Pin, INPUT); // UltraSonic Sensor
  pinMode(Trig_Pin, OUTPUT);

  pinMode(laneKeep_L, INPUT); // Obstacle Avoidance Sensor
  pinMode(laneKeep_R, INPUT);

  // Motor Controller
  pinMode(pinLB,OUTPUT); // /pin 2
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin 6(PWM)   
}

void loop(){
  Obstacle_Avoidance_Main();

}



