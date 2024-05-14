#ifndef Drive_h
#define Drive_h

#include <Arduino.h>
#include <DCmotor.h>
#include <DCmotor.cpp>

class Drive{
public: 
    Drive(DCmotor *fl, DCmotor *fr, DCmotor *bl, DCmotor *br);
    void move_velocity(float steering, float velocity);
    void feedback_callback();
    void move_position(); // this calls position_command for all the drive motors
    void pause_until_position_done();// calls all the pause functions for each motor
private:
    DCmotor *front_left;
    DCmotor *front_right;
    DCmotor *back_left;
    DCmotor *back_right;


};

#endif