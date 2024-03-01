/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-infrared-obstacle-avoidance-sensor
 */

// Arduino's pin connected to OUT pin of IR obstacle avoidance sensor
const int SENSOR_PIN = 8;

int lastState = HIGH;  // the previous state from the input pin
int currentState;      // the current reading from the input pin

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize the Arduino's pin as aninput
  pinMode(SENSOR_PIN, INPUT);
}

void loop() {
  // read the state of the the input pin:
  currentState = digitalRead(SENSOR_PIN);

  if (lastState == HIGH && currentState == LOW)
    Serial.println("The obstacle is detected");
  else if (lastState == LOW && currentState == HIGH)
    Serial.println("The obstacle is cleared");

  delay(50);
  // save the the last state
  lastState = currentState;
}
