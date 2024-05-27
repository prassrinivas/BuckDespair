#include <DCmotor.h>

DCmotor::DCmotor(int pwm, int in1, int in2, int en_pin){
    pwm_pin = pwm;
    dir1_pin = in1;
    dir2_pin = in2;
    encoder_pin = en_pin;
    current_direction = FORWARD;
    encticks = 0;
    encticks_last = 0;
    time_last = 0;
    rpm_setpoint = 0;
    pwm_output = 0;

    pinMode(pwm_pin, OUTPUT);
    pinMode(dir1_pin, OUTPUT);
    pinMode(dir2_pin, OUTPUT);
    pinMode(encoder_pin, INPUT);

    set_dir(current_direction);
};


void DCmotor::enc_tick(){
    if(current_direction == FORWARD)
    encticks++;
    else encticks--;
}
float DCmotor::calc_rpm(){
  float out = (double(encticks - encticks_last)/double(millis() - time_last)) * (MS_PER_MIN/TICKS_PER_REVOLUTION);
  encticks_last = encticks;
  time_last = millis();
  if (current_direction==BACKWARD){
    //Serial.println("negative");
    //out = -out;
  }
  return out;
}

bool DCmotor::send_speed(int pwm){
    if(pwm <0 || pwm > 255) {
        // pwm_output = 0;
        // rpm_setpoint = 0;
        //analogWrite(pwm_pin, 0);
        return false;
    }
    else{
        pwm_output = pwm;
        analogWrite(pwm_pin, pwm);
        //Serial.println(pwm);
        return true;
    }
}
bool DCmotor::set_dir(bool direction){
    current_direction = direction;
    if(direction == FORWARD){
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
        return FORWARD;
    }
    else{
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
        return BACKWARD;
    }
}

bool DCmotor::set_rpm(float rpm){
    mode = VEL_MODE;
    rpm_setpoint = rpm;
    bool change_dir = false;
    if((rpm>=0 && current_direction == BACKWARD) || (rpm<=0 && current_direction == FORWARD)){
        change_dir = true;
    }

    set_dir(rpm >=0);
    int pwm_input = map(((rpm)>0?(rpm):-(rpm)), 0, MAX_SPEED, 0 , 255);
    if(change_dir) pwm_input += 30;
    Serial.println(pwm_input);
    return(send_speed(pwm_input));
}

void DCmotor::position_command(float position){
    mode = POS_MODE;
    encticks = 0;
    pos_setpoint = position; // in turns
    position_command_running = true;
}

bool DCmotor::pause_until_position_done(){

    while(true){
        delay(100);
        if(position_command_running == false) {
            Serial.println("finished");
            break;
        }
    }
    return true;
}


float DCmotor::feedback_callback(){
    //Serial.println(current_direction);
    float cur_rpm = calc_rpm();
    current_position = encticks / TICKS_PER_REVOLUTION;
        if(mode == POS_MODE){
            float pos_error = pos_setpoint - current_position;
            current_direction = (pos_error>=0? (FORWARD): (BACKWARD));
            set_dir(current_direction);
            send_speed(CLAMP(POS_K_GAIN * abs(pos_error),45, 255));
            Serial.println(pos_error);
            if(abs(pos_error) < 0.02){
                //Serial.println("done running");

                position_command_running = false;

            }
        }
            
        else if (mode == VEL_MODE){
            //Serial.println("velocity feedback");
            float control_effort_change = I_GAIN * (rpm_setpoint - cur_rpm);
            integral_term += (current_direction==FORWARD?(control_effort_change):-(control_effort_change));
            pwm_output = 2*(rpm_setpoint - cur_rpm) + integral_term;
            
            send_speed(pwm_output);
            //Serial.println(pwm_output);
        }   
        //Serial.println("end interrupt");

    return cur_rpm;
}


static void disable_timer_interrupt(){
    
  cli();
  TCCR1B &= B11111011;
  TIMSK1 &= B11111101;        
  sei();
}
