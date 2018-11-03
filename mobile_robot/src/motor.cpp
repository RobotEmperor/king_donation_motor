/*
 * motor.cpp
 *
 *      Author: robotemperor
 */
#include <mobile_robot/motor.h>

DcMotorForRaspberryPi::DcMotorForRaspberryPi()
{
}
DcMotorForRaspberryPi::DcMotorForRaspberryPi(int encoder_pulse_per_rotation, int control_freqency, int channel):
    encoder_pulse_per_rotation_(encoder_pulse_per_rotation),
    control_freqency_(control_freqency),
    channel_(channel)
{
  encoder_pulse1 = 0;
  encoder_pulse2 = 0;

  encoder_pulse_position1 = 0;
  encoder_pulse_position2 = 0;

  acceleration_value = 200; //

  p_gain_position_ = 0.01;
  p_gain_speed_ = 0.5;

  pwm_value_motor = 0;

  direction = 0;
  check_position_control= 0;
  onoff = 0;

  speed_motor = 0;
  angle_motor = 0;
  result_rpm = 0;

  speed_static_encoder_pulse_ = 0;
  speed_error_ = 0;
  speed_control_ = 0;

  position_static_encoder_pulse_ = 0;
  position_error_ = 0;
  position_control_ = 0;

  position_max_rpm = 0;
  check_position = true;
}
DcMotorForRaspberryPi::~DcMotorForRaspberryPi()
{

}
void DcMotorForRaspberryPi::speed_controller(int desired_speed)
{
  speed_static_encoder_pulse_ = (encoder_pulse1*2); // digital low pass filter  // basic 2 ch

   //speed_static_encoder_pulse_ = (encoder_pulse1+ encoder_pulse2);
  encoder_pulse1 = 0;
  encoder_pulse2 = 0;
  result_rpm =  (((speed_static_encoder_pulse_)*60*control_freqency_)/(encoder_pulse_per_rotation_*channel_));

 // result_rpm = 


  speed_error_ = desired_speed  -  result_rpm ;
  speed_control_ = ( p_gain_speed_ * speed_error_);

  pwm_value_motor = (pwm_value_motor + speed_control_);

  if (pwm_value_motor > 512)
  {
    pwm_value_motor = 512;
  }

  if (pwm_value_motor < 0)
  {
    pwm_value_motor = 0;
  }

}
double DcMotorForRaspberryPi::position_controller(int desired_angle, int max_rpm)
{
  position_static_encoder_pulse_ = (encoder_pulse_position1+ encoder_pulse_position2);

  if(((desired_angle*encoder_pulse_per_rotation_*channel_)/360) <= position_static_encoder_pulse_)
  {
    check_position = true;
    return 0;
  }
  else
  {
    position_error_ = ((desired_angle*encoder_pulse_per_rotation_*channel_)*360) - position_static_encoder_pulse_;
    position_control_ = p_gain_position_ * position_error_;

    if(position_control_ >  max_rpm)
    {
      position_control_ = max_rpm;
    }
    if(position_control_ < 0)
    {
      position_control_ = 0;
    }
    return position_control_;
  }

}






















