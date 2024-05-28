#include <Drive.h>

Drive::Drive(DCmotor *fl, DCmotor *fr, DCmotor *bl, DCmotor *br):front_left(fl), front_right(fr), back_left(bl), back_right(br){}

void Drive::move_velocity(float steering, float velocity){
    float left_velocity = velocity - steering;
    float right_velocity = velocity + steering;

    front_left->set_rpm(left_velocity);
    back_left ->set_rpm(left_velocity);

    front_right->set_rpm(-right_velocity);
    back_right->set_rpm(-right_velocity);
}

void Drive::move_position(float pos){
    back_left->position_command(pos);
    back_right->position_command(-pos);
    front_right->position_command(-pos);
    front_left->position_command(pos);
}

bool Drive::pause_all_until_position_done() {
    bool blb = back_left->pause_until_position_done();
    bool brb = back_right->pause_until_position_done();
    bool frb = front_right->pause_until_position_done();
    bool flb = front_left->pause_until_position_done();
    return blb && brb && frb && flb;
}

void Drive::turn_90_deg(bool dir) {
    if (dir == true) { //true = right
        move_position(0.15);
        pause_all_until_position_done();
        back_left->set_rpm(50);
        back_right->set_rpm(50);
        front_right->set_rpm(50);
        front_left->set_rpm(50);
        delay(1400);
        back_left->set_rpm(0);
        back_right->set_rpm(0);
        front_right->set_rpm(0);
        front_left->set_rpm(0);
    }
    else {
        move_position(0.15);
        pause_all_until_position_done();
        back_left->set_rpm(-50);
        back_right->set_rpm(-50);
        front_right->set_rpm(-50);
        front_left->set_rpm(-50);
        delay(1400);
        back_left->set_rpm(0);
        back_right->set_rpm(0);
        front_right->set_rpm(0);
        front_left->set_rpm(0);
    }
}

void Drive::feedback_callback(){
    front_left->feedback_callback();
    front_right->feedback_callback();
    back_left->feedback_callback();
    back_right->feedback_callback();

}