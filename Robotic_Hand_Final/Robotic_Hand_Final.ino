#include <Servo.h>
#include <PID_v1.h>

class Finger {
  public:
    int sensorPin;
    int servoPin;
    int from, to;
    double input, output, setpoint;
    double Kp, Ki, Kd;
    PID* pid;
    Servo servo;

    Finger(int sPin, int svPin, double sp, double kp, double ki, double kd, int fr, int t)
      : sensorPin(sPin), servoPin(svPin), setpoint(sp), Kp(kp), Ki(ki), Kd(kd) {
      
      from = fr;
      to = t;
      input = analogRead(sensorPin);
      output = 0;
      pid = new PID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
      pid->SetOutputLimits(0, 180);
      pid->SetMode(AUTOMATIC);
    }

    void attach() {
      servo.attach(servoPin);
    }

    void update() {
      input = analogRead(sensorPin);
      pid->Compute();
      output=map(output,0,255,from,to);
      servo.write(output);
    }
};

Finger middleFinger(A1, 10, 850, 0.8, 0.05, 0.1, 0, 180);   // Middle & Ring (PID)
Finger indexFinger(A3, 5, 890, 1.2, 0.05, 0.1, 180, 0);     // Index (PID)
Finger thumbFinger(A0, 6, 800, 0.8, 0.05, 0.1, 0, 180);     // Thumb (PID)
Finger pinkyFinger(A2, 9, 900, 0.8, 0.05, 0.1, 0, 180);     // pinky (PID)

Servo gripServo;   // Grip (simple servo)

const int buttonPin = 2;  // the number of the pushbutton pin
int buttonState = 0;  // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);

  middleFinger.attach();
  indexFinger.attach();
  thumbFinger.attach();
  pinkyFinger.attach();
  gripServo.attach(3);

  // Startup positions
  middleFinger.servo.write(0);
  indexFinger.servo.write(180);
  thumbFinger.servo.write(180);
  pinkyFinger.servo.write(0);
  gripServo.write(180);  // Half-closed grip
  delay(1500);

  gripServo.write(95);  // Half-closed grip
  delay(1500);

}
void loop() {
  // Update PID-controlled fingers
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
  middleFinger.update();
  indexFinger.update();
  thumbFinger.update();
  pinkyFinger.update();

Serial.print("Middle:");
Serial.print(middleFinger.input);
Serial.print(" Index:");
Serial.print(indexFinger.input);
Serial.print(" Thumb:");
Serial.print(thumbFinger.input);
Serial.print(" Pinky:");
Serial.println(pinkyFinger.input);

  delay(500);

  } else {
  middleFinger.servo.write(0);
  indexFinger.servo.write(180);
  thumbFinger.servo.write(180);
  pinkyFinger.servo.write(0);
  gripServo.write(180);  // Half-closed grip
  delay(1500);

  gripServo.write(95);  // Half-closed grip
  delay(1500);
    }

}
