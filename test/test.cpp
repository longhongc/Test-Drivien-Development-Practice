#include <gtest/gtest.h>
#include "pid.h"

double Kp = 1;
double Ki = 2;
double Kd = 3;
double Ts = 4;
PID pid_test(Kp, Ki, Kd, Ts);

TEST(dummy, should_pass) {
  EXPECT_EQ(1, 1);
}

// Test getters
TEST(PID_class_getters, get_Kp) {
  EXPECT_EQ(pid_test.getPGain(), Kp);
}

TEST(PID_class_getters, get_Ki) {
  EXPECT_EQ(pid_test.getIGain(), Ki);
}

TEST(PID_class_getters, get_Kd) {
  EXPECT_EQ(pid_test.getDGain(), Kd);
}

TEST(PID_class_getters, get_pid) {
  auto pid_gains = pid_test.getPID();
  EXPECT_EQ(pid_gains.at(0), Kp);
  EXPECT_EQ(pid_gains.at(1), Ki);
  EXPECT_EQ(pid_gains.at(2), Kd);
}

TEST(PID_class_getters, get_step_time) {
  EXPECT_EQ(pid_test.getStepTime(), Ts);
}

// Test setters
TEST(PID_class_setters, set_Kp) {
  pid_test.setPGain(5);
  EXPECT_EQ(pid_test.getPGain(), 5);
}

TEST(PID_class_setters, set_Ki) {
  pid_test.setIGain(5);
  EXPECT_EQ(pid_test.getIGain(), 5);
}

TEST(PID_class_setters, set_Kd) {
  pid_test.setDGain(5);
  EXPECT_EQ(pid_test.getDGain(), 5);
}

TEST(PID_class_getters, set_pid) {
  double new_Kp = 7.3;
  double new_Ki = 2.6;
  double new_Kd = 1.5;
  pid_test.setPID(new_Kp, new_Ki, new_Kd);
  EXPECT_EQ(pid_test.getPGain(), new_Kp);
  EXPECT_EQ(pid_test.getIGain(), new_Ki);
  EXPECT_EQ(pid_test.getDGain(), new_Kd);
}

TEST(PID_class_setters, set_step_time) {
  int new_Ts = 10;
  pid_test.setStepTime(new_Ts);
  EXPECT_EQ(pid_test.getStepTime(), new_Ts);
}
