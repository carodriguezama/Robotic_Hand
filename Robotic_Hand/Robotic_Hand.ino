/*
#include <Servo.h>
#include <PID_v1.h>

#define PIN_INPUT A0
#define PIN_OUTPUT 9

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

Servo myservo;//this is a test for one finger

void setup() {
  myservo.attach(PIN_OUTPUT); // Attach each servo to its respective pin

  Input = analogRead(PIN_INPUT);
  Setpoint = 100;//Pressure sensor value 0 to 1024

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  Input = analogRead(PIN_INPUT);

  double gap = abs(Setpoint-Input); //distance away from setpoint
  if (gap < 10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  myPID.Compute();
  Output = map(Output, 0, 255, 0, 180); //go from pwm to an angle
  //analogWrite(PIN_OUTPUT, Output);
  // Move servos from 0 to 180 degrees

  myservo.write(Output);    
  delay(15); // Wait for movement
  

}*/

/*#include <Servo.h>

const int maxServos = 5;
Servo servos[maxServos];  // Create an array of 5 Servo objects
int servoPins[maxServos] = {9, 10, 6, 3, 5}; // Define the pins for each servo

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
/*
#include <Servo.h>

Servo myservo;
int pin = 5;

void setup() {
  myservo.attach(pin);  // 3,6,10 works//9 //3-top back 5-back bottom 6-front bottom 9-front top 10-micro servo
}

void loop() {
  // Move servos from 0 to 180 degrees
   
    myservo.write(0);  // Move each servo to the current position
    delay(15); // Wait for movement

}*/

/*int pin = 3;

void setup() {
  pinMode(pin,OUTPUT);
  //myservo.attach(pin);  // 3,6,10 works//9 and 5 that dont
}

void loop() {
  // Move servos from 0 to 180 degrees
  for (int range = 0; range <= 255; range++) {
    //myservo.write(pos);  // Move each servo to the current position
    analogWrite(pin,range)
    delay(15); // Wait for movement
  }

  // Move servos from 180 to 0 degrees
  for (int range = 255; range >= 0; range--) {
    delay(15); // Wait for movement
  }
}*/
int pin = 5;
void setup() {
  // Set pin 9 as output
  pinMode(pin, OUTPUT);

  // Stop Timer1
  TCCR1A = 0;
  TCCR1B = 0;

  // Set to Fast PWM mode with ICR1 as TOP
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8

  // Set TOP value for 50Hz -> 16MHz / (8 * 50) = 40000
  ICR1 = 40000;
  // Set initial duty cycle (e.g., 1.5 ms pulse width)
  //OCR1A = 3000; // 3000/40000 = 7.5% duty cycle (typical servo center)
  TCCR2A = 0;
  TCCR2B = 0;

  // Set to Fast PWM mode with ICR1 as TOP
  TCCR2A |= (1 << COM1A1) | (1 << WGM11);
  TCCR2B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8

  // Set TOP value for 50Hz -> 16MHz / (8 * 50) = 40000
  ICR2 = 40000;

}

void loop() {
  // You can change OCR1A to control the duty cycle
  analogWrite(pin,4800);
}


