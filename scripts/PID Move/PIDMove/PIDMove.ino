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
 #include <std_msgs/Float64.h>
 #include <std_msgs/String.h> 
 #include <avr/io.h>
 #include <avr/interrupt.h>
 #include <SoftwareSerial.h> 
 

/************************* DEFINES OF CODE**************************/
/*******************************************************************/

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
#define Time5Count 40535

//#define BT_RX 
//#define BT_TX 3 


/************************* GLOBAL VARIABLES*************************/
/*******************************************************************/

ros::NodeHandle nh;
volatile double pwm[4] = {0,0,0,0}; //The motor pwm value between -255->full speed backwards and 255-> Full speed forward
int enables[4] = {MOT1_EN, MOT2_EN, MOT3_EN, MOT4_EN};
int in1[4] = {MOT1_IN1, MOT2_IN1, MOT3_IN1, MOT4_IN1};
int in2[4] = {MOT1_IN2, MOT2_IN2, MOT3_IN2, MOT4_IN2};

//Auxiliar Variables
volatile double w[4] = {0,0,0,0};
volatile double w_abs[4] = {0,0,0,0};
volatile int OldCount[4] = { -999,-999,-999,-999 };
volatile int Count[4];
volatile int interrupts;

char commandReceived = '0';
char text[120];
char pwm1_debug = 'S';
char pwm2_debug = 'S';
double Speeds[4];
boolean ROS_Flag = false;

//PID Library Declaration

PID M1(&Speeds[0],&pwm[0],&w_abs[0], 10,30,1,DIRECT);
PID M2(&Speeds[1],&pwm[1],&w_abs[1], 10,30,1,DIRECT);
PID M3(&Speeds[2],&pwm[2],&w_abs[2], 10,30,1,DIRECT);
PID M4(&Speeds[3],&pwm[3],&w_abs[3], 10,30,1,DIRECT);

//PID Controllers[4] = {M1, M2, M3, M4};
Encoder Motor[4] = {{20,26}, {21,28}, {18,22}, {19,24}};

//Init Bluetooth
//SoftwareSerial BT(BT_RX, BT_TX);   


/************************* CODE LOGIC ******************************/
/*******************************************************************/


/* Stop the motor */
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
void pwm_cb(int motorID) {
    if (w[motorID] >= 0) {
      motor_forward((unsigned int) pwm[motorID], enables[motorID], in1[motorID], in2[motorID]);
    } else if (w[motorID] < 0) {
      motor_backwards((unsigned int) pwm[motorID], enables[motorID], in1[motorID], in2[motorID]);
    }
}

void set_w(int motorID, volatile double value){
  w[motorID] = value;
  w_abs[motorID] = abs(w[motorID]);
  if(w_abs[motorID] > 15){
    w_abs[motorID] = 15;
  }
}

void w0_cb(const std_msgs::Float64 &msg) {
  if(ROS_Flag == false){
    set_w(0,msg.data);
  }
}

void w1_cb(const std_msgs::Float64 &msg) {
  if(ROS_Flag == false){
    set_w(1,msg.data);
  }
}

void w2_cb(const std_msgs::Float64 &msg) {
  if(ROS_Flag == false){
    set_w(2,msg.data);
  }
}

void w3_cb(const std_msgs::Float64 &msg) {
  if(ROS_Flag == false){
    set_w(3,msg.data);
  }
}


void bluetooth(){

  if (Serial3.available()) {
    // Read the incoming letter from BT
    commandReceived = Serial3.read(); 
    
    switch (commandReceived) { //Move forward 
      case '1':
        for(int i = 0; i < 4; i++){
          w_abs[i] = 13.33;
        }     
        break;
      case '2':
        for(int i = 0; i < 4; i++){  //Move backward 
          w_abs[i] = 13.33;
          w[i] = -1;
        }   
        
        break;
      case '3':
        for(int i = 0; i < 4; i++){ //Z angular movement positive
          w_abs[i] = 3.7333;
        }   
          w[0] = -1;
          w[2] = -1;
        
        break;
      case '4':
        for(int i = 0; i < 4; i++){ //Z angular movement negative
          w_abs[i] = 3.7333;
        }   
          w[1] = -1;
          w[3] = -1;
        
        break;
      case '5':
        for(int i = 0; i < 4; i++){ //Y angular movement positive
          w_abs[i] = 13.33;
        }   
          w[0] = -1;
          w[3] = -1;
        
        break;
      case '6':
        for(int i = 0; i < 4; i++){ //Y angular movement negative
          w_abs[i] = 13.33;
        }   
          w[1] = -1;
          w[2] = -1;
        
        break;
      case '7': //Stop
        for(int i = 0; i < 4; i++){
          w_abs[i] = 0;
          w[i] = 0;
        }
        break;
      case '8': //ROS Disable
          ROS_Flag = true;
        break;     
      case '9': //ROS Enable
          ROS_Flag = false;
        break;    
      default:    
      
        break;
    }
  }
}


//Creates the ROS subscribers
ros::Subscriber<std_msgs::Float64> w0_sub("arduino/w0", w0_cb);
ros::Subscriber<std_msgs::Float64> w1_sub("arduino/w1", w1_cb);
ros::Subscriber<std_msgs::Float64> w2_sub("arduino/w2", w2_cb);
ros::Subscriber<std_msgs::Float64> w3_sub("arduino/w3", w3_cb);

std_msgs::Float64 str_msg;
ros::Publisher chatter("chatter", &str_msg); 

ros::Publisher info("info", &str_msg); 

std_msgs::String strmsg;
ros::Publisher speeds("RPM", &strmsg);
char texto[128];
int motorval[2];


/************************* ARDUINO SETUP****************************/
/*******************************************************************/

void setup() {
  
        //init ROS communication
        nh.initNode();
        nh.advertise(chatter);
        nh.advertise(info);
        nh.advertise(speeds);
        
        //Subscribed ROS topics
        nh.subscribe(w0_sub);
        nh.subscribe(w1_sub);
        nh.subscribe(w2_sub);
        nh.subscribe(w3_sub);

        //Init PID Modes
        M1.SetMode(AUTOMATIC);
        M2.SetMode(AUTOMATIC);
        M3.SetMode(AUTOMATIC);
        M4.SetMode(AUTOMATIC);

        //Setup Serial Bps BT
        Serial3.begin(9600);
        Serial.begin(9600);
        
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
        //pinMode(BT_RX, INPUT);
        //pinMode(BT_TX, OUTPUT);


        //Set Timer Logic
        noInterrupts(); //Disable all interrupts
        // Set Timer/Counter Control Register A to 0
        TCCR5A = 0;
        TCCR5B = 0;
        
        TCNT5H = Time5Count/256;
        TCNT5L = Time5Count%256;
        
        // Set prescaler to 64
        TCCR5B |= (0 << CS52) | (1 << CS51) | (1 << CS50);
        
        TIMSK5 |= (1 << TOIE5); // enable timer oveflow interrupt
        interrupts();

}

double CalcVel(Encoder enc, int motorID){
  float delta;
  double vel;
  double RPM;
  Count[motorID] = enc.read();
  if(Count[motorID] != OldCount[motorID]){
    delta = Count[motorID] - OldCount[motorID];
    OldCount[motorID] = Count[motorID];
    vel = 10*(2.0*PI*delta)/CPR;
    RPM = ((vel)/(2.0*PI))*60;
  }
  if (w[motorID]>=0){
    return vel;
  } else {
    return -vel;
  }
  
}

/*********************TIMER INTERRUPT 10HZ**************************/
/*******************************************************************/

ISR(TIMER5_OVF_vect) {
  // Code to execute when the timer interrupt occurs
  //double Speeds[4];
    bluetooth();
    for(int i = 0; i < 4; i++){
        Speeds[i] = CalcVel(Motor[i], i);
    }
    
    M1.Compute();
    M2.Compute();
    M3.Compute();
    M4.Compute();
  
    for(int i = 0; i < 4; i++){
        pwm_cb(i);
    }
    
    TCNT5H = Time5Count/256;
    TCNT5L = Time5Count%256;
    interrupts++;   
}

/************************* ARDUINO LOOP*****************************/
/*******************************************************************/

void loop() {

        //ROS required functions
        nh.spinOnce();
        std_msgs::Float64 aux;
        aux.data = Speeds[1];
       
        //Bluetooth Arduino Logic
        
               
        //Debug info to ROS
        chatter.publish(&aux);
        aux.data = pwm[1];
        info.publish(&aux);
        sprintf(text, "M0 = %d M1 = %d M2 = %d M3 = %d", (int)(Speeds[0]*10.0), (int)(Speeds[1]*10.0),(int)(Speeds[2]*10.0),(int)(Speeds[3]*10.0));
        strmsg.data = text;
        speeds.publish(&strmsg);
        delay(200);
        
  }
