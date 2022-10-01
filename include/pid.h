#ifndef PID_H
#define PID_H
#include <vector>

/**
 * @Brief  A simple PID implementation
 */
class PID {
 public:
   /**
    * @Brief  Constructor
    *
    * @Param p P gain value
    * @Param i I gain value
    * @Param d D gain value
    * @Param ts The sampling time or step time for
    *           each iteration.
    */
   PID(double p, double i, double d, int ts):
       m_Kp{p}, m_Ki{i}, m_Kd{d}, m_Ts{ts}{}

   /**
    * @Brief  Empty constructor
    *         Starting with a default value of 
    *         p: 0, i: o, d: 0, ts: 1
    */
   PID(){ PID(0, 0, 0, 1); }

   /**
    * @Brief  Setter for m_Kp
    *
    * @Param p P gain value
    */
   void setPGain(double p);
   /**
    * @Brief  Setter for m_Ki
    *
    * @Param i I gain value
    */
   void setIGain(double i);
   /**
    * @Brief  Setter for m_Kd
    *
    * @Param d D gain value
    */
   void setDGain(double d);
   /**
    * @Brief Setter for PID gains
    *
    * @Param p Value for m_Kp
    * @Param i Value for m_Ki
    * @Param d Value for m_Kd
    */
   void setPID(double p, double i, double d);

   /**
    * @Brief  Setter for m_Ts
    *
    * @Param t The sampling time or step time for
    *          each iteratoin.
    */
   void setStepTime(int t); 

   /**
    * @Brief  Getter for m_Kp
    *
    * @Returns m_Kp
    */
   double getPGain();
   /**
    * @Brief  Getter for m_Ki
    *
    * @Returns m_Ki
    */
   double getIGain();
   /**
    * @Brief  Getter for m_Kd
    *
    * @Returns m_Kd
    */
   double getDGain();
   /**
    * @Brief  Getter for PID gains
    *
    * @Returns a vector that stores PID gains in sequence of P,I,D
    */
   std::vector<double> getPID();
   /**
    * @Brief  Getter for m_Ts
    *
    * @Returns m_Ts  
    */
   int getStepTime(); 

   /**
    * @Brief  Calculate the PID control
    *
    * @Param target The setpoint that wanted to be achieved
    * @Param input The input signal
    *
    * @Returns The output signal
    *          calculated by adding PID control to the input signal
    */
   double calcOutput(double target, double input);

 private:
   /**
    * @Brief  The proportional gain in pid control.
    *         Proportional control updates the input by adding
    *         the product of this param with the error
    *         between the target and input.
    *         ouput += Kp * (target - input)
    */
   double m_Kp;
   /**
    * @Brief  The integral gain in pid control.
    *         Integral control updates the input by adding
    *         the product of this param with the sum of
    *         all past errors and the current error
    *         multiply by sampling time.
    *         ouput += Ki * (total_past_errors + current_error
    *                                         * sampling_time)
    */
   double m_Ki;
   /**
    * @Brief  The derivative gain in pid control.
    *         Derivative control updates the input by adding
    *         the product of this param with the change of error
    *         (The velocity of error).
    *         output += Kd * (current_error - prev_error)
    */
   double m_Kd;

   /**
    * @Brief The sampling time or step time.
    *        Ki and Kd will be updated with respect to
    *        this sampling time.
    */
   int m_Ts;

   /**
    * @Brief  The vector collects the error between input and target.
    */
   std::vector<double> past_errors;
};

#endif

