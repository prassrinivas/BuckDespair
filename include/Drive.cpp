#include <Drive.h>


Drive::Drive(DCmotor *fl, DCmotor *fr, DCmotor *bl, DCmotor *br):front_left(fl), front_right(fr), back_left(bl), back_right(br){}

void Drive::move_velocity(float steering, float velocity){
    float left_velocity = velocity - steering;
    float right_velocity = velocity + steering;

    front_left->set_rpm(left_velocity);
    back_left ->set_rpm(left_velocity);

    front_right->set_rpm(right_velocity);
    back_right->set_rpm(right_velocity);
}

void Drive::feedback_callback(){
    front_left->feedback_callback();
    front_right->feedback_callback();
    back_left->feedback_callback();
    back_right->feedback_callback();

}