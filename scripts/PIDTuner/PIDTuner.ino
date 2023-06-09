#include "Arduino.h"
#include <ros.h>
#include <TimeLib.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h> 
#include <avr/io.h>
#include <avr/interrupt.h>


#define MOT1_IN1 48//IN1 of the L298 should be connected to this arduino pin
#define MOT1_IN2 46//IN2 of the L298 should be connected to this arduino pin
#define MOT2_IN1 52//IN3 of the L298 should be connected to this arduino pin
#define MOT2_IN2 50//IN4 of the L298 should be connected to this arduino pin
#define MOT4_IN1 44//IN1 of the L298 should be connected to this arduino pin
#define MOT4_IN2 42//IN2 of the L298 should be connected to this arduino pin
#define MOT3_IN1 40//IN3 of the L298 should be connected to this arduino pin
#define MOT3_IN2 38//IN4 of the L298 should be connected to this arduino pin
#define MOT1_EN 13 //enable
#define MOT2_EN 12 //enable
#define MOT4_EN 11
#define MOT3_EN 10

#define ENCODER_MOT1a 20
#define ENCODER_MOT1b 26

#define ENCODER_MOT2a 21
#define ENCODER_MOT2b 28 

#define ENCODER_MOT3a 18
#define ENCODER_MOT3b 22

#define ENCODER_MOT4a 19
#define ENCODER_MOT4b 24


#define CPR 4480
#define Time5Count 63306//40535

int enables[4] = {MOT1_EN, MOT2_EN, MOT3_EN, MOT4_EN};
int in1[4] = {MOT1_IN2, MOT2_IN2, MOT3_IN2, MOT4_IN2};
int in2[4] = {MOT1_IN1, MOT2_IN1, MOT3_IN1, MOT4_IN1};
volatile double pwm[4] = {0,0,0,0}; //The motor pwm value between -255->full speed backwards and 255-> Full speed forward
double Speeds[4];
volatile double w_abs[4] = {0,0,0,0};
volatile int OldCount[4] = { -999,-999,-999,-999 };
volatile int Count[4];
volatile double w[4] = {0,0,0,0};

PID Controllers[4] = {{&Speeds[0],&pwm[0],&w_abs[0], 10,30,0,DIRECT},
                      {&Speeds[1],&pwm[1],&w_abs[1], 10,30,0,DIRECT},
                      {&Speeds[2],&pwm[2],&w_abs[2], 10,30,0,DIRECT},
                      {&Speeds[3],&pwm[3],&w_abs[3], 10,30,0,DIRECT}};
                      
                      

Encoder Motor[4] = {{20,26}, {21,28}, {18,22}, {19,24}};

ISR(TIMER5_OVF_vect) {
  // Code to execute when the timer interrupt occurs
  //double Speeds[4];
    for(int i = 0; i < 4; i++){
        Speeds[i] = CalcVel(Motor[i], i);
    }
    
    for(int i = 0; i < 4; i++){
      Controllers[i].Compute();
    }
  
    for(int i = 0; i < 4; i++){
        pwm_cb(i);
    }
    
    TCNT5H = Time5Count/256;
    TCNT5L = Time5Count%256;
    //interrupts++;   
}

double CalcVel(Encoder enc, int motorID){
  float delta;
  double vel;
  double RPM;
  Count[motorID] = -1*enc.read(); //Inverts the value of the encoder, since it cant be changed when assigning pins.
  if(Count[motorID] != OldCount[motorID]){
    delta = Count[motorID] - OldCount[motorID];
    OldCount[motorID] = Count[motorID];
    vel = 100*(2.0*PI*delta)/CPR;
    RPM = ((vel)/(2.0*PI))*60;
  }else
    vel = 0;
  if (w[motorID]>=0){
    return vel;
  } else {
    return -vel;
  }
  
}


void motor_stop(int EN, int In1, int In2) {
        //Stop if received an wrong direction
                digitalWrite(In2, 0);
                digitalWrite(In1, 0);
                analogWrite(EN, 0);
               
}


void motor_forward(unsigned int vel, int EN, int In1, int In2) {
        digitalWrite(In1, 1);
        digitalWrite(In2, 0);
        analogWrite(EN, vel);
       
}



/* Moves the motor backwards
        vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_backwards(unsigned int vel, int EN, int In1, int In2) {
    digitalWrite(In1, 0);
    digitalWrite(In2, 1);
    analogWrite(EN, vel);
    
}


//Motor callback
void pwm_cb(int motorID) {

    if (w_abs[motorID] == 0.0){
        motor_stop(enables[motorID], in1[motorID], in2[motorID]);
    }
    else{
      if (w[motorID] >= 0) {
        motor_forward((unsigned int) pwm[motorID], enables[motorID], in1[motorID], in2[motorID]);
      } else if (w[motorID] < 0) {
        motor_backwards((unsigned int) pwm[motorID], enables[motorID], in1[motorID], in2[motorID]);
      }
    }
}


void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  for(int i = 0; i < 4; i++){
    Controllers[i].SetMode(AUTOMATIC);
  }
  pinMode(MOT1_IN1, OUTPUT);
  pinMode(MOT1_IN2, OUTPUT);
  pinMode(MOT1_EN, OUTPUT);
  pinMode(MOT2_IN1, OUTPUT);
  pinMode(MOT2_IN2, OUTPUT);
  pinMode(MOT2_EN, OUTPUT);
  pinMode(MOT3_IN1, OUTPUT);
  pinMode(MOT3_IN2, OUTPUT);
  pinMode(MOT3_EN, OUTPUT);
  pinMode(MOT4_IN1, OUTPUT);
  pinMode(MOT4_IN2, OUTPUT);
  pinMode(MOT4_EN, OUTPUT);

  TCCR5A = 0;
  TCCR5B = 0;
  
  TCNT5H = Time5Count/256;
  TCNT5L = Time5Count%256;
  
  // Set prescaler to 64
  TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50);
  
  TIMSK5 |= (1 << TOIE5); // enable timer oveflow interrupt
  for(int i = 0; i < 4; i++){
    w_abs[i] = 0;
  }
  delay(1000);
}

void SetVars(PID* Controller, int k){
 
 char* information;
 char* Token;
 double Params[3];
 char comma[2] = ",";
 information = new char[Serial.available()+1];
 int sizeB = Serial.available();
 for(int i = 0; i < sizeB; i++){
    information[i] = Serial.read();
    
 }
 information[sizeB+1] = '\0';
 //Serial.print(information); Serial.print("\t");
 Token = strtok(information,comma);
 //Serial.print(Token); Serial.print("\t");
 Params[0] = atof(Token);
 Token = NULL;
 for(int i = 1; i < 3; i++){
   Token = strtok(NULL,comma);
   //Serial.print(Token); Serial.print("\t");
   Params[i] = atof(Token);
   Token = NULL;
 }
 //for(int i = 0; i < 3; i++){
 // Serial.print(Params[i]);
 // Serial.print("\t");
 //}
 //Serial.println(" ");
 Controller->SetTunings(Params[0],Params[1],Params[2]);
 free(information);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(Serial.available()){
    char numbers[4];
    numbers[3] = '\0';
    int i = 0;
    
    char val = Serial.read();
    switch(val){
      case '5':
        while(Serial.available() && i < 3){
          numbers[i] = Serial.read();
          i++;
        }
        break;
       case '1':
          SetVars(&Controllers[0], 0);
        break;
       case '2':
          SetVars(&Controllers[1], 1);
        break;
       case '3':
          SetVars(&Controllers[2], 2);
        break;
       case '4':
          SetVars(&Controllers[3], 3);
        break;
    }  
    
    float res = atof(numbers);
    for(int i = 0; i < 4; i++){
      w_abs[i] = res;
    }
  }
  //pwm_cb(0);
  //pwm_cb(1); 
  //pwm_cb(2);
  //pwm_cb(3); 
  
  Serial.print("Kp:"); Serial.print((int)(Controllers[0].GetKp()));
  Serial.print("\t");
  Serial.print("Ki:"); Serial.print((int)(Controllers[0].GetKi()));
  Serial.print("\t");
  Serial.print("Kd:"); Serial.print((int)(Controllers[0].GetKd()));
  
  Serial.print("\t");
  Serial.print("Motor1:"); Serial.print((int)(10*Speeds[0]));
  Serial.print("\t");
  Serial.print("Motor2:");Serial.print((int)(10*Speeds[1]));
  Serial.print("\t");
  Serial.print("Motor3:");Serial.print((int)(10*Speeds[2]));
  Serial.print("\t");
  Serial.print("Motor4:");Serial.print((int)(10*Speeds[3]));
  Serial.print("\t");
  Serial.print("Setpoint:");Serial.println((int)(10*w_abs[0]));
  
  delay(25);
}
