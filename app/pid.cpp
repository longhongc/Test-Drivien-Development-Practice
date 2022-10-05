#include <numeric>
#include "pid.h"

void PID::setPGain(double p) {
  m_Kp = p;
  return;
}

void PID::setIGain(double i) {
  m_Ki = i;
  return;
}

void PID::setDGain(double d) {
  m_Kd = d;
  return;
}

void PID::setPID(double p, double i, double d) {
  m_Kp = p;
  m_Ki = i;
  m_Kd = d;
  return;
}

void PID::setStepTime(int t) {
  m_Ts = t;
  return;
}

double PID::getPGain() {
  return m_Kp;
}

double PID::getIGain() {
  return m_Ki;
}

double PID::getDGain() {
  return m_Kd;
}

std::vector<double> PID::getPID() {
  std::vector<double> pids = {m_Kp,m_Ki,m_Kd};
  return pids;
}

int PID::getStepTime() {
  return m_Ts;
}

std::vector<double> PID::getPastErrors() {
  return m_past_errors;
}

void PID::clearPastErrors() {
  m_past_errors.clear();
  return;
}

double PID::calcOutput(double target, double input) {
  double e_current = target-input;
  
  double rate_of_e = 0.0;
  if (!m_past_errors.empty()){
    rate_of_e = e_current - m_past_errors.back();
  }

  double area_of_e = std::accumulate(m_past_errors.begin(),m_past_errors.end(),0.0);

  double new_velocity = m_Kp*e_current + m_Ki*(area_of_e + e_current*m_Ts) + m_Kd*rate_of_e;

  m_past_errors.push_back(e_current);
  return new_velocity;
}
