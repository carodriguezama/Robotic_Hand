#include "Finger.h"

Finger::Finger(int inputPin, int outputPin, double setpointVal)
  : pinInput(inputPin), pinOutput(outputPin),
    aggKp(4), aggKi(0.2), aggKd(1),
    consKp(1), consKi(0.05), consKd(0.25),
    pid(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT)
{
  Setpoint = setpointVal;
}

void Finger::begin() {
  servo.attach(pinOutput);
  Input = analogRead(pinInput);
  pid.SetMode(AUTOMATIC);
}

void Finger::update() {
  Input = analogRead(pinInput);

  double gap = abs(Setpoint - Input);
  if (gap < 10) {
    pid.SetTunings(consKp, consKi, consKd);
  } else {
    pid.SetTunings(aggKp, aggKi, aggKd);
  }

  pid.Compute();
  Output = map(Output, 0, 255, 0, 180);
  servo.write(Output);
  delay(15);
}

void Finger::setSetpoint(double sp) {
  Setpoint = sp;
}
