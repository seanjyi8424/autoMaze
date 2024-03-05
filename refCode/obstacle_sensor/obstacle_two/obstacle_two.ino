/*
 * Code by Artip Nakchinda
 * Inspiration from ArduinoGetStarted.com
 * Tutorial page https://arduinogetstarted.com/tutorials/arduino-infrared-obstacle-avoidance-sensor
 *
 */

// Arduino's pin connected to OUT pin of IR obstacle avoidance sensor
// oSensor - Obstacle Avoidance Sensor
const int OSENSOR_R = 11; // right
const int OSENSOR_L = 12; // left

int lastState_L = HIGH;  // the previous state from the input pin
int currState_L;      // the current reading from the input pin

int lastState_R = HIGH;
int currState_R;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  // initialize the Arduino's pin as an input
  pinMode(OSENSOR_R, INPUT);
  pinMode(OSENSOR_L, INPUT);
}

void loop() {
    // read the state of the the input pin:
    currState_L = digitalRead(OSENSOR_L);
    currState_R = digitalRead(OSENSOR_R);

    if (lastState_L == HIGH && currState_L == LOW)
        Serial.println("Line not detected on the left");
    else if (lastState_L == LOW && currState_L == HIGH)
        Serial.println("Line detected on the left");

    if (lastState_R == HIGH && currState_R == LOW)
        Serial.println("Line not detected on the right");
    else if (lastState_R == LOW && currState_R == HIGH)
        Serial.println("Line detected on the right");

    if ((currState_L == HIGH && currState_R == HIGH) 
        && (lastState_L == LOW && lastState_R == LOW))
            Serial.println("Line detected in front");

    delay(500);

    // save the the last state
    lastState_L = currState_L;
    lastState_R = currState_R;
}
