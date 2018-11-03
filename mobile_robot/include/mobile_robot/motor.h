/*
 * motor.h
 *
 *      Author: robotemperor
 */
#include <stdio.h>
#include <math.h>

class DcMotorForRaspberryPi
{
  public:
    DcMotorForRaspberryPi();
    DcMotorForRaspberryPi(int encoder_pulse_per_rotation, int control_freqency, int ch);
    ~DcMotorForRaspberryPi();

   // int position_control(int* encoder_read_position, int desired_position, int max_out_put, bool* check);
    double pwm_value_motor;
    double speed_motor;
    double angle_motor;
    double result_rpm;
    bool direction;
    bool check_position_control;
    bool onoff;

    int encoder_pulse1;
    int encoder_pulse2;

    int encoder_pulse_position1;
    int encoder_pulse_position2;

    int position_max_rpm;
    bool check_position;

    double acceleration_value;

    void speed_controller(int desired_speed);
    double position_controller(int desired_angle, int max_rpm);

  private:
    int encoder_pulse_per_rotation_;
    int control_freqency_;
    int channel_;

    double p_gain_position_;
    double p_gain_speed_;

    double speed_static_encoder_pulse_;
    double speed_error_;
    double speed_control_;

    double position_static_encoder_pulse_;
    double position_error_;
    double position_control_;
};

