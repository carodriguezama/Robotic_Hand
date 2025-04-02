#include <Servo.h>
Servo myservo;  // create Servo object to control a servo
Servo myservo2;  // create Servo object to control a servo
Servo myservo3;  // create Servo object to control a servo
Servo myservo4;
Servo myservo5;
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  myservo2.attach(10);
  myservo3.attach(11);
  myservo4.attach(3);
  myservo5.attach(5);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);
    myservo3.write(pos);
    myservo4.write(pos);
    myservo5.write(pos);
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);
    myservo3.write(pos);
    myservo4.write(pos);
    myservo5.write(pos);
    delay(5);                       // waits 15 ms for the servo to reach the position
  }
}
/*
#include <Servo.h>
int maxServos = 5;
Servo servos[maxServos];  // Create an array of 5 Servo objects
int servoPins[maxServos] = {9, 10, 11, 12, 13}; // Define the pins for each servo

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
}*/
