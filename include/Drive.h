#ifndef Drive_h
#define Drive_h

#include <Arduino.h>
#include <DCmotor.h>

class Drive{
public: 
    Drive(DCmotor *fl, DCmotor *fr, DCmotor *bl, DCmotor *br);
    void move_velocity(float steering, float velocity);
    void feedback_callback();
    void move_position(float pos); // this calls position_command for all the drive motors
    bool pause_all_until_position_done();// calls all the pause functions for each motor
    void turn_90_deg(bool dir);
private:
    DCmotor *front_left;
    DCmotor *front_right;
    DCmotor *back_left;
    DCmotor *back_right;


};

#endif