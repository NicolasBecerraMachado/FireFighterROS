

/*
 * This program receives a ROS (Int32) message (between -255 and 255)
 * and commands a DC motor according to the specified value.
 * subscribed ROS topic arduino/pwm1
 */

 #include "Arduino.h"
 #include <ros.h>
 #include <TimeLib.h>
 #include <Encoder.h>
 #include <PID_v1.h>
 #include <std_msgs/Int32.h>
 #include <std_msgs/String.h> 
 #include <avr/io.h>
 #include <avr/interrupt.h>


/***********
 * Define the material
 **********/
//Arduino pin definitions
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

#define LED 7 //A led that blinks when receiving

#define CPR 4480

/*
 * Global variables
 */
ros::NodeHandle nh;
volatile int pwm1; //The motor pwm value between -255->full speed backwards and 255-> Full speed forward
volatile int pwm2; //The motor pwm value between -255->full speed backwards and 255-> Full speed forward
volatile int pwm3;
volatile int pwm4;

volatile int OldCount[4] = { -999,-999,-999,-999 };
volatile int Count[4];
volatile int interrupts;

char pwm1_debug = 'S';
char pwm2_debug = 'S';
int compareMatchReg;
unsigned long t;
unsigned long oldt;
boolean interrupt0;
unsigned int reload = 0xF424; 
double Speeds[4];
//PID M1(&Speeds[0],&pwm1,&w0, 1,0,0,DIRECT);

Encoder Motor[4] = {{20,26}, {21,28}, {18,22}, {19,24}};

/* 
 * Stop the motor
 */
void motor_stop(int EN, int In1, int In2) {
        //Stop if received an wrong direction
                digitalWrite(In2, 0);
                digitalWrite(In1, 0);
                analogWrite(EN, 0);
                pwm1_debug = 'S';
}



/* Moves the motor in positive sens
        vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_forward(unsigned int vel, int EN, int In1, int In2) {
        digitalWrite(In1, 1);
        digitalWrite(In2, 0);
        analogWrite(EN, vel);
        pwm1_debug = 'F';
}



/* Moves the motor backwards
        vel: [0-255] the desired pwm value 255 means full speed
 */
void motor_backwards(unsigned int vel, int EN, int In1, int In2) {
    digitalWrite(In1, 0);
    digitalWrite(In2, 1);
    analogWrite(EN, vel);
    pwm1_debug = 'B';
}


//Motor callback
void pwm1_cb(const std_msgs::Int32 & msg) {
        digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led to indicate this topic is active
        pwm1 = msg.data;
        if (pwm1 >= 0 && pwm1 <= 255) {
                motor_forward((unsigned int) pwm1, MOT1_EN, MOT1_IN1, MOT1_IN2);
        } else if (pwm1 < 0 && pwm1 >= -255) {
                motor_backwards((unsigned int) -1*pwm1, MOT1_EN, MOT1_IN1, MOT1_IN2);
        } else {
                //Stop if received a wrong value or 0
                nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
                nh.logerror("The motor will stop");
                motor_stop(MOT1_EN, MOT1_IN1, MOT1_IN2);
        }
}

void pwm2_cb(const std_msgs::Int32 & msg) {
        digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led to indicate this topic is active
        pwm2 = msg.data;
        if (pwm2 >= 0 && pwm2 <= 255) {
                motor_forward((unsigned int) pwm2, MOT2_EN, MOT2_IN1, MOT2_IN2);
        } else if (pwm2 < 0 && pwm2 >= -255) {
                motor_backwards((unsigned int) -1*pwm2, MOT2_EN, MOT2_IN1, MOT2_IN2);
        } else {
                //Stop if received a wrong value or 0
                nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
                nh.logerror("The motor will stop");
                motor_stop(MOT2_EN, MOT2_IN1, MOT2_IN2);
        }
}

void pwm3_cb(const std_msgs::Int32 & msg) {
        digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led to indicate this topic is active
        pwm3 = msg.data;
        if (pwm3 >= 0 && pwm3 <= 255) {
                motor_forward((unsigned int) pwm3, MOT3_EN, MOT3_IN1, MOT3_IN2);
        } else if (pwm3 < 0 && pwm3 >= -255) {
                motor_backwards((unsigned int) -1*pwm3, MOT3_EN, MOT3_IN1, MOT3_IN2);
        } else {
                //Stop if received a wrong value or 0
                nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
                nh.logerror("The motor will stop");
                motor_stop(MOT3_EN, MOT3_IN1, MOT3_IN2);
        }
}

void pwm4_cb(const std_msgs::Int32 & msg) {
        digitalWrite(LED, HIGH - digitalRead(LED)); //toggles a led to indicate this topic is active
        pwm4 = msg.data;
        if (pwm4 >= 0 && pwm4 <= 255) {
                motor_forward((unsigned int) pwm4, MOT4_EN, MOT4_IN1, MOT4_IN2);
        } else if (pwm4 < 0 && pwm4 >= -255) {
                motor_backwards((unsigned int) -1*pwm4, MOT4_EN, MOT4_IN1, MOT4_IN2);
        } else {
                //Stop if received a wrong value or 0
                nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
                nh.logerror("The motor will stop");
                motor_stop(MOT4_EN, MOT4_IN1, MOT4_IN2);
        }
}


//Creates the ROS subscribers
ros::Subscriber<std_msgs::Int32> pwm1_sub("arduino/pwm1", pwm1_cb);

//Creates the ROS subscribers
ros::Subscriber<std_msgs::Int32> pwm2_sub("arduino/pwm2", pwm2_cb);

ros::Subscriber<std_msgs::Int32> pwm3_sub("arduino/pwm3", pwm3_cb);

//Creates the ROS subscribers
ros::Subscriber<std_msgs::Int32> pwm4_sub("arduino/pwm4", pwm4_cb);

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg); 

ros::Publisher info("info", &str_msg); 
char texto[128];
int motorval[2];
/*
 * Arduino SETUP
 */
void setup() {
        //init ROS communication
        nh.initNode();
        nh.advertise(chatter);
        nh.advertise(info);
        //Subscribed ROS topics
        nh.subscribe(pwm1_sub);
        nh.subscribe(pwm2_sub);
        nh.subscribe(pwm3_sub);
        nh.subscribe(pwm4_sub);
        // Arduino pins definition
        pinMode(LED, OUTPUT);
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

        noInterrupts(); //Disable all interrupts
        // Set Timer/Counter Control Register A to 0
        TCCR5A = 0;
        TCCR5B = 0;

        //compareMatchReg = (16000 / (1024 *5)) - 1;
        TCNT5 = 65535-24999;
        
        // Set prescaler to 1024
        TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50);
        
        TIMSK5 |= (1 << TOIE5); // enable timer oveflow interrupt
        interrupts();
        /*
        // Calculate and set timer period
        //float desired_delay = 1; // 1 second delay
        //int timer_period = (int)((1.0 / (16000000.0 / 1024.0)) * desired_delay);
        TCNT1 = 12499  ;
        OCR1A = reload;
        
        // Enable timer interrupt
        TIMSK1 |= (1 << OCIE1A);
        
        // Enable interrupts
        sei();*/

}

double CalcVel(Encoder enc, int motorID){
  float delta;
  double w;
  double RPM;
  Count[motorID] = enc.read();
  if(Count[motorID] != OldCount[motorID]){
    delta = Count[motorID] - OldCount[motorID];
    OldCount[motorID] = Count[motorID];
    w = (2.0*PI*delta)/CPR;
    RPM = 6.82*((w)/(2.0*PI)*60);
  }
    sprintf(texto, "MotorID = %d  OldCount = %d Delta = %d W = %d RPM = %d", motorID,OldCount[motorID],(int)delta, (int)w, (int)RPM);
    str_msg.data = texto; 
    info.publish( &str_msg );
    
  return RPM;
}


ISR(TIMER5_OVF_vect) {
  // Code to execute when the timer interrupt occurs
  //double Speeds[4];
  for(int i = 0; i < 4; i++){
      Speeds[i] = CalcVel(Motor[i], i);
  }
  sprintf(texto, "M0 = %d M1 = %d M2 = %d M3 = %d", (int)Speeds[0], (int)Speeds[1],(int)Speeds[2],(int)Speeds[3]);
  str_msg.data = texto;
  chatter.publish( &str_msg );
  TCNT5  = 65535-24999;
  interrupts++;   
  
}
/*
 * Arduino MAIN LOOP
 */
void loop() {
        nh.spinOnce();
        
        //motorval[0] = analogRead(A1);
        //motorval[1] = analogRead(A0);
        
        
    
        
        
        //delay(1);
        
  }
