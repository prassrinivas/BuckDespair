#include <Arduino.h>
#include <DCmotor.h>
#include <DCmotor.cpp>
#include <AccelStepper.h>

#define pwm 8
#define in1 24
#define in2 25
#define encoderA 3

#define pwm_2 10
#define in1_2 22
#define in2_2 23
#define encoderA_2 2

#define pwm_3 13
#define in1_3 38
#define in2_3 39
#define encoderA_3 19

float current_rpm = 0;
int i = 0;
DCmotor mot1(pwm, in1, in2, encoderA);
DCmotor mot2(pwm_2, in1_2, in2_2, encoderA_2);
DCmotor mot3(pwm_3, in1_3, in2_3, encoderA_3);
DCmotor mot4(7,40,41,18);
AccelStepper stepper1(1, 22, 23); // (Type of driver: with 2 pins, STEP, DIR)
AccelStepper stepper2(AccelStepper::DRIVER,24,25);
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

void mot1_enc(){
  mot1.enc_tick();
}

void mot2_enc(){
  //Serial.println("encoder");
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
  attachInterrupt(digitalPinToInterrupt(encoderA), mot1_enc, RISING);
  attachInterrupt(digitalPinToInterrupt(2), mot2_enc, RISING);
  attachInterrupt(digitalPinToInterrupt(19), mot3_enc, RISING);
  attachInterrupt(digitalPinToInterrupt(18), mot4_enc, RISING);


  //stepper1.setMaxSpeed(1200);
                     //Enable back the interrupts

  Serial.begin(9600);
  Serial.println("setup");
  enable_timer_interrupt();
  //mot1.set_rpm(80);
  //mot1.set_dir(BACKWARD);
  //mot2.set_rpm(80);
  digitalWrite(LED_BUILTIN, LED_STATE);
  //stepper1.setAcceleration(1500);
  //stepper1.setCurrentPosition(0);
  //stepper2.setMaxSpeed(1000);
  //stepper2.setAcceleration(2000);
  //stepper2.setCurrentPosition(0);

  // mot1.position_command(-1);
  // mot2.position_command(1);
  // mot3.position_command(-1);
  // mot4.position_command(1);
  //mot2.pause_until_position_done();
  

  //Serial.println("done");

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
  //mot1.position_command(-1);
  //mot2.position_command(-1);
  //mot3.position_command(-1);
  //mot4.position_command(-1);
  mot4.set_dir(BACKWARD);
  //mot1.pause_until_position_done();
  //mot2.pause_until_position_done();
  //mot3.pause_until_position_done();
  //mot4.pause_until_position_done();


  //mot1.position_command(1);
  //mot2.position_command(1);
  //mot3.position_command(1);
  //mot4.position_command(1);

  //mot1.pause_until_position_done();
  //mot2.pause_until_position_done();
  //mot3.pause_until_position_done();
  //mot4.pause_until_position_done();
}
  
ISR(TIMER1_COMPA_vect){
  //Serial.println("interrupt");
  OCR1A = TCNT1 + TIMER_INTERVAL;          
  LED_STATE = !LED_STATE;      //Invert LED state
  digitalWrite(LED_BUILTIN,LED_STATE);  //Write new state to the LED on pin D5
  mot1.feedback_callback();
  mot2.feedback_callback();
  mot3.feedback_callback();
  mot4.feedback_callback();
}