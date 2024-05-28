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
#define SCREW_GRABBER_ENABLE 41 

#define OPEN true
#define CLOSED false

#define OUT true
#define IN false

float current_rpm = 0;
int i = 0;

/////CHANGE THESE/////
bool type = CHICKEN;
int drop_off = 4;
//////////////////////

int drop_off_count = 0; 

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
AccelStepper grabber(1, 5, 31); // (Type of driver: with 2 pins, STEP, DIR)
AccelStepper screw(1, 6, 32); // (Type of driver: with 2 pins, STEP, DIR)

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
  digitalWrite(CAM_ENABLE,HIGH);
  switch(position){
    case 1: 
      cam.moveTo(0);
      break;
    case 2: 
      cam.moveTo(200);
      break;
    case 3: 
      cam.moveTo(400);
      break;
  }
  cam.runToPosition();
  digitalWrite(CAM_ENABLE, LOW);
}

void actuate_grabber(bool position){
  digitalWrite(SCREW_GRABBER_ENABLE, HIGH);
  switch(position){
    case OPEN:
      grabber.moveTo(400);
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
        screw.moveTo(1600);

        break;
      case IN:
        screw.moveTo(0);
        break;
  }
  screw.runToPosition();
  digitalWrite(SCREW_GRABBER_ENABLE, LOW);

}
void pick_up_sequence() {
  //open claw
  //extend claw
  //close claw
  //retract claw
  //lower plate
}

void drop_off_sequence() {
  //raise plate
  //extend claw
  //open claw
  //retract claw
  //close claw
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
  digitalWrite(CAM_ENABLE, LOW);
  cam.setMaxSpeed(1000);
  cam.setAcceleration(750);
  cam.setCurrentPosition(0);
  pinMode(SCREW_GRABBER_ENABLE, OUTPUT);
  digitalWrite(SCREW_GRABBER_ENABLE, LOW);

  screw.setMaxSpeed(1000);
  screw.setAcceleration(250);
  screw.setCurrentPosition(0);


  grabber.setMaxSpeed(1000);
  grabber.setAcceleration(500);
  grabber.setCurrentPosition(0);


  delay(5000);
  Serial.println("done");
}

void loop() {
// stepper2.moveTo(-5*100);
// stepper2.runToPosition();
// Serial.println("reached");
// delay(1000);
// stepper2.moveTo(0);
// stepper2.runToPosition();
// Serial.println("reached 2");
 //  delay(1000);

  // AWD.move_position(0.5);

  // AWD.pause_all_until_position_done();

  // AWD.move_position(-0.5);

  // AWD.pause_all_until_position_done();

  // AWD.turn_90_deg(true);
  // AWD.turn_90_deg(false);

  // if (i%1000 == 0) {
  //   Serial.print("Motor 1 rpm: ");
  //   Serial.println(mot1.calc_rpm());
  //   Serial.print("Motor 2 rpm: ");
  //   Serial.println(mot2.calc_rpm());
  //   Serial.print("Motor 3 rpm: ");
  //   Serial.println(mot3.calc_rpm());
  //   Serial.print("Motor 4 rpm: ");
  //   Serial.println(mot4.calc_rpm());
  // }

  // i = i+1;
//////CODE START/////
  //Serial.println("start of loop");

  // int IRread_middle = digitalRead(IRpin_middle);
  // int IRread_middle_right = digitalRead(IRpin_middle_right); 
  // int IRread_middle_left = digitalRead(IRpin_middle_left); 
  // int IRread_right = digitalRead(IRpin_right); 
  // int IRread_left = digitalRead(IRpin_left); 
  // //if (IRread_middle == 1) {FUCKTHEMIDDLEONEFUCKTHEMIDDLEONEFUCKTHEMIDDLEONEFUCKTHEMIDDLEONEFUCKTHEMIDDLEONEFUCKTHEMIDDLEONEFUCKTHEMIDDLEONE
  //   //Serial.println("in first if");
  //   if (IRread_right == 0 && IRread_left == 0) {
  //     Serial.println("Not at intersection!");
  //     if (IRread_middle_left == 0 && IRread_middle_right == 0) { //go straight
  //       Serial.println("Centered, moving forward!");
  //       AWD.move_velocity(0,40);
  //       delay(50);
  //       AWD.move_velocity(0,0);
  //     }
  //     else if (IRread_middle_left == 1 && IRread_middle_right == 0) { //adjust left
  //       Serial.println("Too far right, adjusting left");
  //       //Adjust left function
  //       AWD.move_velocity(10,0);
  //       delay(100);
  //       AWD.move_velocity(0,0);
  //     }
  //     else if (IRread_middle_left == 0 && IRread_middle_right == 1) { //adjust right
  //       Serial.println("Too far left, adjusting right!");
  //       //Adjust right function
  //       AWD.move_velocity(-10,0);
  //       delay(100);
  //       AWD.move_velocity(0,0);
  //     }
  //   }
  //   else if ((IRread_right == 1 && IRread_middle_right == 0)){
  //     Serial.println("Fairly diagonal, adjusting right");
  //     mot1.position_command(0.1);
  //     mot4.position_command(0.1);
  //     mot1.pause_until_position_done();
  //     mot4.pause_until_position_done();
  //   }
  //   else if ((IRread_left == 1 && IRread_middle_left == 0)){
  //     Serial.println("Fairly diagonal, adjusting left");
  //     mot2.position_command(0.1);
  //     mot3.position_command(0.1);
  //     mot2.pause_until_position_done();
  //     mot3.pause_until_position_done();
  //   }

  //   else if (IRread_right == 1 && IRread_left == 1) { //4-way intersection
  //     Serial.println("At a 4 way intersection!");
  //     if (type == CHICKEN) {
  //       Serial.println("Picking up Chicken!");
  //       //Rotate Left Function
  //       AWD.turn_90_deg(false);
  //       float dis = measure_ultrasonic();
  //       Serial.println(dis);
  //       float rot_to_wall = 0.75*dis/22.9399;
  //       AWD.move_position(rot_to_wall);
  //       AWD.pause_all_until_position_done();
  //       Serial.println("moved to wall");
  //       pick_up_sequence();
  //       Serial.println("picked up disk");
  //       AWD.move_position(-rot_to_wall);
  //       AWD.pause_all_until_position_done();
  //       Serial.println("Moved away from wall");
  //       //rotate right function
  //       AWD.turn_90_deg(true);
  //       Serial.println("turned back to track");
  //       drop_off_count = 0;
  //     }
  //     else {
  //       Serial.println("Picking up Beef!");
  //       //Rotate Right Function
  //       AWD.turn_90_deg(true);
  //       float dis = measure_ultrasonic();
  //       float rot_to_wall = 0.75*dis/22.9399;
  //       AWD.move_position(rot_to_wall);
  //       AWD.pause_all_until_position_done();
  //       pick_up_sequence();
  //       AWD.move_position(-rot_to_wall);
  //       AWD.pause_all_until_position_done();
  //       //rotate left function
  //       AWD.turn_90_deg(false);
  //       drop_off_count = 0;
  //     }
  //   }
  //   else if (IRread_right == 0 && IRread_left == 1) { //left intersection
  //     Serial.println("At Left Turn!");
  //     //Turn left code
  //     AWD.turn_90_deg(false);
  //   }
  //   else if (IRread_right == 1 && IRread_left == 0) { //right intersection
  //     Serial.println("At Right Turn!");
  //     drop_off_count++;
  //     if (drop_off_count == drop_off) {
  //       //Rotate Right Function
  //       AWD.turn_90_deg(true);
  //       float dis = measure_ultrasonic();
  //       float rot_to_wall = 0.75*dis/22.9399;
  //       AWD.move_position(rot_to_wall);
  //       AWD.pause_all_until_position_done();
  //       drop_off_sequence();
  //       AWD.move_position(-rot_to_wall);
  //       AWD.pause_all_until_position_done();
  //       AWD.turn_90_deg(false);
  //       //rotate left function
  //     }
  //     else {
  //       Serial.println("move until passed line");
  //       AWD.move_velocity(0,10);
  //       delay(500);
  //       AWD.move_velocity(0,0);
  //     }
  //   }
  // //}
  // //else {

  //   if (IRpin_middle_right == 1) {
  //     //adjust left
  //     Serial.println("middle sensor off, adjusting right");
  //     AWD.move_velocity(-10,0);
  //     delay(250);
  //     AWD.move_velocity(0,0);
  //   }
  //   else if (IRpin_middle_left == 1) {
  //     Serial.println("middle sensor off, adjusting left");
  //     AWD.move_velocity(10,0);
  //     delay(250);
  //     AWD.move_velocity(0,0);
  //   }
  actuate_cam(1);
  actuate_cam(2);
  actuate_cam(3);
  actuate_grabber(OPEN);
  actuate_grabber(CLOSED);
  actuate_screw(OUT);
  actuate_screw(IN);

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


  
  // Serial.print(rpm1);
  // Serial.print(", ");
  // Serial.print(rpm2);
  // Serial.print(", ");
  // Serial.print(rpm3);
  // Serial.print(", ");
  // Serial.print(rpm4);
  // Serial.print("\n");
}