#ifndef FINGER_H
#define FINGER_H

#include <Servo.h>
#include <Arduino.h>
#include <PID_v1.h>

class Finger {
  private:
    int pinInput;
    int pinOutput;

    double Setpoint, Input, Output;

    double aggKp, aggKi, aggKd;
    double consKp, consKi, consKd;

    PID pid;
    Servo servo;

  public:
    Finger(int inputPin, int outputPin, double setpointVal = 100);

    void begin();
    void update();
    void setSetpoint(double sp);
};

#endif // MYFINGER_H
