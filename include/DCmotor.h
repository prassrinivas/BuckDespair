#ifndef dcmot_h
#define dcmot_h
#include <Arduino.h>

#define TICKS_PER_REVOLUTION 900.00//540.00
#define MS_PER_MIN 60000.00
#define MAX_SPEED 70
#define FORWARD 1
#define BACKWARD 0
#define I_GAIN 0.5
#define TIMER_INTERVAL 2500
#define POS_K_GAIN 4250

#define IDLE_MODE 1
#define VEL_MODE 2
#define POS_MODE 3

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(x, low, high) (MIN(MAX(x, low), high))

class DCmotor{
public:
    DCmotor(int pwm, int in1, int in2, int en_pin);
    void enc_tick();
    float calc_rpm();
    bool send_speed(int pwm);
    bool set_rpm(float rpm);
    void position_command(float position);
    bool set_dir(bool direction);
    float feedback_callback();
    bool pause_until_position_done();

    int mode = VEL_MODE;
    int pwm_pin = 0;
    int dir1_pin = 0;
    int dir2_pin = 0;
    int encoder_pin = 0;
    float pos_setpoint = 0;
    float current_position = 0;
    bool current_direction = FORWARD;
    bool position_command_running = false;

    float rpm_setpoint = 0;
    float integral_term = 0.0;
    int pwm_output = 0;
    volatile long encticks = 0;
    volatile long encticks_last = 0;
    volatile long time_last = 0;
};

#endif