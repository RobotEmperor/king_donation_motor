/*
 * motor_algorithm.cpp
 *
 *      Author: robotemperor
 */
#include <mobile_robot/motor_algorithm.h>
#include <mobile_robot/motor_cmd.h>

//////////////////////////////////////////////////////////////////////////////
TrajectoryGenerator::TrajectoryGenerator()
{
  pre_desired_value = 0;
  current_desired_value = 0;
  out_value = 0;
  time_count = 0;
  tra_done_check = 0;

}
TrajectoryGenerator::~TrajectoryGenerator()
{

}
double TrajectoryGenerator::linear_function(double desired_value, double acceleration)
{
  static double time = 0;

  if(current_desired_value != desired_value)
  {
    time_count = 0;
    tra_done_check = false;
    pre_desired_value = out_value;
  }

  time = fabs((pre_desired_value - desired_value) / acceleration);


  current_desired_value = desired_value;

  time_count = time_count + 0.01;

  if(time_count >= time)
  {
    tra_done_check = true;
    pre_desired_value = desired_value;
    return pre_desired_value;
  }

  if(pre_desired_value != desired_value && tra_done_check == false)
  {
    if(pre_desired_value > desired_value) // 하강 트레젝토리 y = -at + b
    {
      out_value = -(acceleration)*time_count + pre_desired_value;
      return out_value;
    }
    if(pre_desired_value < desired_value)// 상승 트레젝토리 y = at + b
    {
      out_value = (acceleration)*time_count + pre_desired_value;
      return out_value;
    }
  }
  else
  {
    return pre_desired_value;
  }

}
//////////////////////////////////////////////////////////////////////////////
void initialize()
{
  reference_angle = 0;
  reference_distance = 0;

  motor1 = new DcMotorForRaspberryPi(156,100,2);
  motor2 = new DcMotorForRaspberryPi(156,100,2);

  tra_motor1 = new TrajectoryGenerator;
  tra_motor2 = new TrajectoryGenerator;

  current_desired_speed_motor1 = 0;
  current_desired_speed_motor2 = 0;

  motor1->check_position = true;  
  motor2->check_position = true;
}
/*void motor1_encoder_1(void)
{
  motor1->encoder_pulse1 ++;
  motor1->encoder_pulse_position1 ++;
}
void motor1_encoder_2(void)
{
  motor1->encoder_pulse2++;
  motor1->encoder_pulse_position2 ++;
}

void motor2_encoder_1(void)
{
  motor2->encoder_pulse1 ++;
  motor2->encoder_pulse_position1 ++;
}
void motor2_encoder_2(void)
{
  motor2->encoder_pulse2 ++;
  motor2->encoder_pulse_position2 ++;
}*/
//
void motor_first_command_callback(const mobile_robot::motor_cmd::ConstPtr& msg)
{
  motor1->speed_motor = msg->motor_desired_speed;
  motor1->onoff       = msg->motor_onoff;
}
void motor_second_command_callback(const mobile_robot::motor_cmd::ConstPtr& msg)
{
  motor2->speed_motor = msg->motor_desired_speed;
  motor2->onoff       = msg->motor_onoff;
}
/////////////////////////////////////////////////////////////////////////////////////
void motor_control(int pwm_pin, int direction_pin, int mode, int desired_speed_rpm, int angle, bool on_off, DcMotorForRaspberryPi* motor, TrajectoryGenerator* tra_value)
{
  if(on_off == true)
  {
    motor->pwm_value_motor = tra_value->linear_function(desired_speed_rpm, motor->acceleration_value);
    if(motor->pwm_value_motor > 0)
    {
      digitalWrite(direction_pin,LOW); //ccw
      motor->direction = 0;
    }
    else
    {
      digitalWrite(direction_pin,HIGH); //cw
      motor->direction = 1;
    }
  }
  if(motor->pwm_value_motor < 0)
    motor->pwm_value_motor = motor->pwm_value_motor*-1;

  if(on_off == false || desired_speed_rpm <= 10)
  {
   pwmWrite(pwm_pin, 0);
  }
}

void controlFunction(const ros::TimerEvent&)
{

  motor1->onoff = 1;
  motor2->onoff = 1;

  motor_control(motor1_PWM, motor1_DIR, 0, motor1->speed_motor, 0, motor1->onoff, motor1, tra_motor1);
  motor_control(motor2_PWM, motor2_DIR, 0, motor2->speed_motor, 0, motor2->onoff, motor2, tra_motor2);
}
int main (int argc, char **argv)
{

  printf("Motor node Start \n");
  wiringPiSetupGpio();

  initialize();

  ros::init(argc, argv, "motor_node");
  ros::NodeHandle nh;

  ros::Timer timer_control = nh.createTimer(ros::Duration(0.01), controlFunction); // 10ms

  std::string motor_first_topic = nh.param<std::string>("motor_first", "");
  std::string motor_second_topic = nh.param<std::string>("motor_second", "");

  motor_first_command_sub   = nh.subscribe(motor_first_topic, 1, motor_first_command_callback);
  motor_second_command_sub   = nh.subscribe(motor_second_topic, 1, motor_second_command_callback);

  pinMode(motor1_IN1, OUTPUT);
  pinMode(motor1_BREAK, OUTPUT);
  pinMode(motor1_DIR, OUTPUT);
  pinMode(motor1_FG1, INPUT);

  pinMode(motor2_IN1, OUTPUT);
  pinMode(motor2_BREAK, OUTPUT);
  pinMode(motor2_DIR, OUTPUT);
  pinMode(motor2_FG1, INPUT);


 // wiringPiISR(motor1_FG1, INT_EDGE_RISING, &motor1_encoder_1);
 // wiringPiISR(motor2_FG1, INT_EDGE_RISING, &motor2_encoder_1);

  pinMode(motor1_PWM, PWM_OUTPUT);
  pinMode(motor2_PWM, PWM_OUTPUT);
  pwmSetMode (PWM_MODE_MS);
  pwmSetRange(512);
  pwmSetClock(45);

  digitalWrite(motor1_IN1,LOW);
  digitalWrite(motor1_BREAK,LOW);
  digitalWrite(motor2_IN1,LOW);
  digitalWrite(motor2_BREAK,LOW);


  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  while(ros::ok())
  {
    pwmWrite(motor1_PWM, (int) motor1->pwm_value_motor);
    pwmWrite(motor2_PWM, (int) motor2->pwm_value_motor);

    printf("motor1->pwm_value_motor:: %f \n", motor1->pwm_value_motor);
    printf("motor2->pwm_value_motor:: %f \n", motor2->pwm_value_motor);
    usleep(100);
    ros::spinOnce();
  }

  delete motor1;
  delete motor2;
  delete tra_motor1;
  delete tra_motor2;
  pwmWrite(motor1_PWM, 0);
  pwmWrite(motor2_PWM, 0);

  return 0;
}


