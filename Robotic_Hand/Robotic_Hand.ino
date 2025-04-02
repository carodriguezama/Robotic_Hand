#include <Servo.h>

const int maxServos = 5;
Servo servos[maxServos];  // Create an array of 5 Servo objects
int servoPins[maxServos] = {9, 10, 11, 3, 5}; // Define the pins for each servo

void setup() {
  for (int i = 0; i < maxServos; i++) {
    servos[i].attach(servoPins[i]);  // Attach each servo to its respective pin
  }
}

void loop() {
  // Move servos from 0 to 180 degrees
  for (int pos = 0; pos <= 180; pos++) {
    for (int i = 0; i < maxServos; i++) {
      servos[i].write(pos);  // Move each servo to the current position
    }
    delay(15); // Wait for movement
  }

  // Move servos from 180 to 0 degrees
  for (int pos = 180; pos >= 0; pos--) {
    for (int i = 0; i < maxServos; i++) {
      servos[i].write(pos);  // Move each servo to the current position
    }
    delay(15); // Wait for movement
  }
}
