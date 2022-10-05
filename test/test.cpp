#include <gtest/gtest.h>
#include "pid.h"
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

// Test PID calculation

/**
 * @Brief Test the p control value
 *
 */
TEST(PID_calculation, calc_P) {
  double new_Kp = 2;
  double new_Ki = 0;
  double new_Kd = 0;
  pid_test.setPID(new_Kp, new_Ki, new_Kd);

  double target = 2;
  double input = 1;
  auto p_control_value = new_Kp * (target - input);
  EXPECT_EQ(pid_test.calcOutput(target, input), p_control_value);
}

/**
 * @Brief  Test the I control value for several iterations
 *
 */
TEST(PID_calculation, calc_I) {
  pid_test.clearPastErrors();
  double new_Kp = 0;
  double new_Ki = 0.5;
  double new_Kd = 0;
  pid_test.setPID(new_Kp, new_Ki, new_Kd);
  int new_Ts = 1;
  pid_test.setStepTime(new_Ts);

  double target = 9;
  double input = 1;

  // iteration 1
  double error_1 = target - input;
  auto u1 = new_Ki * (error_1 * new_Ts);
  EXPECT_EQ(pid_test.calcOutput(target, input), u1);

  // iteration 2
  input += u1;
  double error_2 = target - input;
  auto u2 = new_Ki * (error_1 + error_2 * new_Ts);
  EXPECT_EQ(pid_test.calcOutput(target, input), u2);

  // check past errors
  auto past_errors = pid_test.getPastErrors();
  ASSERT_EQ(int(past_errors.size()), 2);
  EXPECT_EQ(past_errors.at(0), error_1);
  EXPECT_EQ(past_errors.at(1), error_2);

  pid_test.clearPastErrors();
  past_errors = pid_test.getPastErrors();
  ASSERT_EQ(int(past_errors.size()), 0);
}

/**
 * @Brief  Test the D control value for several iterations
 *
 */
TEST(PID_calculation, calc_D) {
  pid_test.clearPastErrors();
  double new_Kp = 0.5;
  double new_Ki = 0;
  double new_Kd = 0.5;
  pid_test.setPID(new_Kp, new_Ki, new_Kd);
  int new_Ts = 1;
  pid_test.setStepTime(new_Ts);

  double target = 5;
  double input = 1;

  // iteration 1, including p control and d control
  double error_1 = target - input;
  auto u1 = new_Kp * error_1 + new_Ki * 0;
  EXPECT_EQ(pid_test.calcOutput(target, input), u1);

  // iteration 2
  input += u1;
  double error_2 = target - input;
  auto u2 = new_Kp * error_2 + new_Ki * (error_2 - error_1);
  EXPECT_EQ(pid_test.calcOutput(target, input), u2);

  // check past errors
  auto past_errors = pid_test.getPastErrors();
  ASSERT_EQ(int(past_errors.size()), 2);
  EXPECT_EQ(past_errors.at(0), error_1);
  EXPECT_EQ(past_errors.at(1), error_2);

  pid_test.clearPastErrors();
  past_errors = pid_test.getPastErrors();
  ASSERT_EQ(int(past_errors.size()), 0);
}


