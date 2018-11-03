/*
 * motor_algorithm.h
 *
 *      Author: robotemperor
 */

//pin information

#define motor1_BREAK 22 // Break
#define motor1_IN1   5 // Enable
#define motor1_DIR   3

#define motor1_PWM   13 // pwm
#define motor1_FG1 9 // FG



#define motor2_BREAK 27 // Break
#define motor2_IN1   6 // Enable
#define motor2_DIR   4

#define motor2_PWM   18 // pwm
#define motor2_FG1 10 // FG


#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <wiringPi.h>

//ros_communication_message type
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>

//custom header
#include <mobile_robot/motor_cmd.h>
#include <mobile_robot/motor.h>

class TrajectoryGenerator
{
  public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();
    double linear_function(double desired_value, double acceleration);

  private:
    double pre_desired_value;
    double current_desired_value;
    double out_value;
    double time_count;
    bool   tra_done_check;

};


//ros communication

ros::Subscriber motor_first_command_sub;
ros::Subscriber motor_second_command_sub;

DcMotorForRaspberryPi* motor1;
DcMotorForRaspberryPi* motor2;

TrajectoryGenerator* tra_motor1;
TrajectoryGenerator* tra_motor2;

double current_desired_speed_motor1;
double current_desired_speed_motor2;

double reference_angle;
double reference_distance;

//function
void initialize();
void algorithm(double angle, double distance);
void motor_control(int pwm_pin, int direction_pin, int mode, int desired_speed_rpm, int angle, bool on_off, DcMotorForRaspberryPi* motor, TrajectoryGenerator* tra_value);

void motor_first_command_callback(const mobile_robot::motor_cmd::ConstPtr& msg);
void motor_second_command_callback(const mobile_robot::motor_cmd::ConstPtr& msg);

//timer
void controlFunction(const ros::TimerEvent&);






