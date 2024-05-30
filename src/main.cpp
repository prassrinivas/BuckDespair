#include <Arduino.h>
#include <DCmotor.h>
#include <AccelStepper.h>
#include <Drive.h>

#define CHICKEN true
#define BEEF false

#define pwm 9
#define in1 24
#define in2 25
#define encoderA 3

#define pwm_2 10
#define in1_2 34
#define in2_2 35
#define encoderA_2 2

#define pwm_3 8
#define in1_3 22
#define in2_3 23
#define encoderA_3 19

#define pwm_4 7
#define in1_4 36
#define in2_4 37
#define encoderA_4 18

#define IRpin_middle 50
#define IRpin_middle_right 51 
#define IRpin_middle_left 49 
#define IRpin_right 52
#define IRpin_left 48 

#define CAM_ENABLE 40
#define SCREW_GRABBER_ENABLE 41 //turns the grabber and lead screw steppers on/off (digital)

#define OPEN true
#define CLOSED false

#define OUT true
#define IN false

float current_rpm = 0;
int i = 0;

/////CHANGE THESE/////
bool type = CHICKEN;
int drop_off = 2;
//////////////////////

int drop_off_count = 0; 
int pick_up_count = 0;
int stack = 1;

const int trigPin = 43;
const int echoPin = 44;

float duration, distance;

DCmotor mot1(pwm, in1, in2, encoderA);
DCmotor mot2(pwm_2, in1_2, in2_2, encoderA_2);
DCmotor mot3(pwm_3, in1_3, in2_3, encoderA_3);
DCmotor mot4(pwm_4,in1_4,in2_4,encoderA_4);
DCmotor* mot1_pointer = &mot1;
DCmotor* mot2_pointer = &mot2;
DCmotor* mot3_pointer = &mot3;
DCmotor* mot4_pointer = &mot4;
Drive AWD(mot4_pointer, mot3_pointer, mot1_pointer, mot2_pointer);
AccelStepper cam(1, 4, 30); // (Type of driver: with 2 pins, STEP, DIR)
AccelStepper grabber(1, 6, 31); // (Type of driver: with 2 pins, STEP, DIR)
AccelStepper screw(1, 5, 28); // (Type of driver: with 2 pins, STEP, DIR)

//AccelStepper stepper2(AccelStepper::DRIVER,24,25);
bool LED_STATE = LOW;


static void enable_timer_interrupt(){
 cli();                      //stop interrupts for till we make the settings
  TCCR1A = 0;                 // Reset entire TCCR1A to 0 
  TCCR1B = 0;                 // Reset entire TCCR1B to 0
  TCCR1B |= B00000100;        //Set CS12 to 1 so we get prescalar 256  
  TIMSK1 |= B00000010;        //Set OCIE1A to 1 so we enable compare match A 
  OCR1A = TIMER_INTERVAL;             //Finally we set compare register A to this value  
  sei(); 
}


void disable_timer_interrupt(){
    
  cli();
  TCCR1B &= B11111011;
  TIMSK1 &= B11111101;        
  sei();
}


float measure_ultrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  return distance;
}

void actuate_cam(int position){
  //digitalWrite(CAM_ENABLE,HIGH);
  switch(position){
    case 0: 
      cam.moveTo(0);
      break;
    case 1: 
      cam.moveTo(250);
      break;
    case 2: 
      cam.moveTo(550);
      break;
    case 3: 
      cam.moveTo(800);
      break;
    default:
      break;
  }
  cam.runToPosition();
  //digitalWrite(CAM_ENABLE, LOW);
}

void actuate_grabber(bool position){
  digitalWrite(SCREW_GRABBER_ENABLE, HIGH);
  switch(position){
    case OPEN:
      grabber.moveTo(-300);
      break;
    case CLOSED:
      grabber.moveTo(0);
      break;
  }
  grabber.runToPosition();
  digitalWrite(SCREW_GRABBER_ENABLE, LOW);

}

void actuate_screw(bool position){
  digitalWrite(SCREW_GRABBER_ENABLE, HIGH);

  switch(position){
      case OUT:
        screw.moveTo(35000);
        Serial.print("out");

        break;
      case IN:
        screw.moveTo(0);
        Serial.print("in");
        break;
  }
  screw.runToPosition();
  digitalWrite(SCREW_GRABBER_ENABLE, LOW);

}
void pick_up_sequence(int stack) {
  //drop the cam
  actuate_grabber(OPEN);
  actuate_cam(stack);
  //open claw
  //extend claw
  actuate_screw(OUT);
  //close claw
  actuate_grabber(CLOSED);
  //retract claw
  actuate_screw(IN);

  AWD.move_position(-0.2);
  AWD.pause_all_until_position_done();
  actuate_grabber(OPEN);
  actuate_cam(0);
  actuate_grabber(CLOSED);

  AWD.move_position(0.2);
  AWD.pause_all_until_position_done();
}

void drop_off_sequence() {
  //raise plate
  actuate_grabber(OPEN);

  actuate_cam(0);
  actuate_grabber(CLOSED);
  //extend claw
  actuate_screw(OUT);
  //open claw
  actuate_grabber(OPEN);
  //retract claw
  actuate_screw(IN);
  //close claw
  actuate_grabber(CLOSED);
}

void mot1_enc(){
  mot1.enc_tick();
}

void mot2_enc(){
  mot2.enc_tick();
}

void mot3_enc(){
  mot3.enc_tick();
}

void mot4_enc(){
  mot4.enc_tick();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(IRpin_middle,INPUT); 
  pinMode(IRpin_middle_right,INPUT); 
  pinMode(IRpin_middle_left,INPUT); 
  pinMode(IRpin_right,INPUT); 
  pinMode(IRpin_left,INPUT); 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA), mot1_enc, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_2), mot2_enc, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_3), mot3_enc, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA_4), mot4_enc, RISING);

                     //Enable back the interrupts

  Serial.begin(9600);
  Serial.println("setup");
  enable_timer_interrupt();

  digitalWrite(LED_BUILTIN, LED_STATE);

  //STEPPER SETUP
  pinMode(CAM_ENABLE,OUTPUT);
  digitalWrite(CAM_ENABLE, HIGH);
  cam.setMaxSpeed(500);
  cam.setAcceleration(250);
  cam.setCurrentPosition(0);
  pinMode(SCREW_GRABBER_ENABLE, OUTPUT);
  digitalWrite(SCREW_GRABBER_ENABLE, LOW);

  screw.setMaxSpeed(2000);
  screw.setAcceleration(1000);
  screw.setCurrentPosition(0);


  grabber.setMaxSpeed(1000);
  grabber.setAcceleration(500);
  grabber.setCurrentPosition(0);


  delay(3000);
  Serial.println("done");
}

void loop() {
//////CODE START/////
start:
  int IRread_middle_right = digitalRead(IRpin_middle_right); 
  int IRread_middle_left = digitalRead(IRpin_middle_left); 
  int IRread_right = digitalRead(IRpin_right); 
  int IRread_left = digitalRead(IRpin_left); 
  if (IRread_right == 0 && IRread_left == 0) {
    Serial.println("Not at intersection!");
    if (IRread_middle_left == 0 && IRread_middle_right == 0) { //go straight
      Serial.println("Centered, moving forward!");
      AWD.move_velocity(0,40);
      delay(50);
      AWD.move_velocity(0,0);
    }
    else if (IRread_middle_left == 1 && IRread_middle_right == 0) { //adjust left
      Serial.println("Too far right, adjusting left");
      //Adjust left function
      // AWD.steer_position(-0.05);
      // AWD.pause_all_until_position_done();
      AWD.move_velocity(20,0);
      delay(100);
      AWD.move_velocity(0,0);
    }
    else if (IRread_middle_left == 0 && IRread_middle_right == 1) { //adjust right
      Serial.println("Too far left, adjusting right!");
      //Adjust right function
      // AWD.steer_position(0.05);
      // AWD.pause_all_until_position_done();
      AWD.move_velocity(-20,0);
      delay(100);
      AWD.move_velocity(0,0);
    }
  }
  else if (IRread_right == 1 && IRread_left == 1) { //4-way intersection
    fourway:
    //delay(3000);
    if(pick_up_count == 2){
      AWD.move_position(0.1);
      AWD.pause_all_until_position_done();
      Serial.println("GOIN BACK TO THE START");
      goto start;
    }
    Serial.println("At a 4 way intersection!");
    pick_up_count++;
    if (type == CHICKEN) {
      Serial.println("Picking up Chicken!");
      //Rotate Left Function
      AWD.turn_90_deg(false);
      float dis = measure_ultrasonic();
      //Serial.println(dis);
      float rot_to_wall = 0.75*dis/22.9399;
      float rot_from_wall = -1.15*dis/22.9399;
      AWD.move_position(rot_to_wall);
      AWD.pause_all_until_position_done();
      //Serial.println("moved to wall");
      stack++;
      pick_up_sequence(stack);
      //Serial.println("picked up disk");
      AWD.move_position(rot_from_wall);
      AWD.pause_all_until_position_done();
      //Serial.println("Moved away from wall");
      //rotate right function
      AWD.turn_90_deg(true);
      AWD.move_position(0.05);
      AWD.pause_all_until_position_done();
      //Serial.println("turned back to track");
    }
    else {
      Serial.println("Picking up Beef!");
      //Rotate Right Function
      AWD.turn_90_deg(true);
      float dis = measure_ultrasonic();
      float rot_to_wall = 0.75*dis/22.9399;
      float rot_from_wall = -1.15*dis/22.9399;
      AWD.move_position(rot_to_wall);
      AWD.pause_all_until_position_done();
      stack++;
      pick_up_sequence(stack);
      AWD.move_position(rot_from_wall);
      AWD.pause_all_until_position_done();
      //rotate left function
      AWD.turn_90_deg(false);
      AWD.move_position(0.05);
      AWD.pause_all_until_position_done();

    }
  }
  else if (IRread_right == 0 && IRread_left == 1) { //left intersection
    Serial.println("At Left Turn!");
    //delay(1000);
    AWD.move_position(0.03);
    AWD.pause_all_until_position_done();
    IRread_right = digitalRead(IRpin_right); 
    if (IRread_right == 1) {
      Serial.println("goto fourway");
      goto fourway;
    }
    else {
      AWD.move_position(-0.03);
      AWD.pause_all_until_position_done();

    }
    Serial.println("confirm left turn");

    //Turn left code
    AWD.turn_90_deg(false);
  }
  else if (IRread_right == 1 && IRread_left == 0) { //right intersection
  Serial.println("At Right Turn!");
  //delay(1000);
    AWD.move_position(0.03);
    AWD.pause_all_until_position_done();
    IRread_left = digitalRead(IRpin_left); 
    if (IRread_left == 1) {
      Serial.println("goto fourway");
      goto fourway;
    }
    else {
      AWD.move_position(-0.03);
      AWD.pause_all_until_position_done();
    }
    Serial.println("confirm right turn");
    drop_off_count++;
    if (drop_off_count == drop_off) {
      //Rotate Right Function
      AWD.turn_90_deg(true);
      float dis = measure_ultrasonic();
      float rot_to_wall = 0.7*dis/22.9399;
      float rot_from_wall = -1.05*dis/22.9399;
      AWD.move_position(rot_to_wall);
      AWD.pause_all_until_position_done();
      drop_off_sequence();
      AWD.move_position(rot_from_wall);
      AWD.pause_all_until_position_done();
      AWD.turn_90_deg(false);
      AWD.move_position(0.05);
      AWD.pause_all_until_position_done();
      //rotate left function
    }
    else {
      Serial.println("move until passed line");
      AWD.move_position(0.1);
      AWD.pause_all_until_position_done();
    }
  }
}
  
ISR(TIMER1_COMPA_vect){
  //Serial.println("interrupt");
  OCR1A = TCNT1 + TIMER_INTERVAL;          
  LED_STATE = !LED_STATE;      //Invert LED state
  digitalWrite(LED_BUILTIN,LED_STATE);  //Write new state to the LED on pin D5
  float rpm1 = mot1.feedback_callback();
  float rpm2 = mot2.feedback_callback();
  float rpm3 = mot3.feedback_callback();
  float rpm4 = mot4.feedback_callback();
}