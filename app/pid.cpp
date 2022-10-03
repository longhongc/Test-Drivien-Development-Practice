#include "pid.h"

void PID::setPGain(double p) {
  return;
}

void PID::setIGain(double i) {
  return;
}

void PID::setDGain(double d) {
  return;
}

void PID::setPID(double p, double i, double d) {
  return;
}

void PID::setStepTime(int t) {
  return;
}

double PID::getPGain() {
  return 0.0;
}

double PID::getIGain() {
  return 0.0;
}

double PID::getDGain() {
  return 0.0;
}

std::vector<double> PID::getPID() {
  return {};
}

int PID::getStepTime() {
  return 0;
}

std::vector<double> PID::getPastErrors() {
  return {};
}

void PID::clearPastErrors() {
  return;
}

double PID::calcOutput(double target, double input) {
  return 0.0;
}
