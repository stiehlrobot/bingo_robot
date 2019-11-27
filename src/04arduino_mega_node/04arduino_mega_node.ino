#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <Arduino.h>

// define max and min range values for PWM

#define PWMRANGE 255
#define PWM_MIN 0

//define pins for right encoder channel A and B inputs

const byte rightMotorEncoderPinA = 21; // Arduino mega pin 20 for encoder output A
const byte rightMotorEncoderPinB = 20; // Arduino mega pin 21 for encoder output B


//define pins for left encoder channel A and B inputs

const byte leftMotorEncoderPinA = 19; // Arduino mega pin 19 for encoder output A
const byte leftMotorEncoderPinB = 18; // Arduino mega pin 18 for encoder output B

volatile long rightCount = 0; //arduino reference recommends using volatile type when working with counters for interrupts
volatile long leftCount = 0; //arduino reference recommends using volatile type when working with counters for interrupts

#define readA digitalRead(rightMotorEncoderPinA)
#define readB digitalRead(rightMotorEncoderPinB)

#define readC digitalRead(leftMotorEncoderPinA)
#define readD digitalRead(leftMotorEncoderPinB)

//OLD
 #define outputRightA 40
 #define outputRightB 41
 #define outputLeftA 42
 #define outputLeftB 43

//OLD
//define counters for 

int leftCounter = 0;
int rightCounter = 0;

//OLD
//define variables for holding encoder states for left and right encoders

int rightAState;
int leftAState;
int rightALastState;  
int leftALastState;

//define motor PINS used by Elecrow 2-Channel H-bridge Arduino Shield 

const byte IN1_RIGHT = 4;
const byte IN2_RIGHT = 5;
const byte PWM_RIGHT = 9;

const byte IN1_LEFT = 7;
const byte IN2_LEFT = 8;
const byte PWM_LEFT = 10;

//

int STBY = 6; //standby
bool move_forward_left = true;
bool move_forward_right = true;


//functions definitions
void handle_Twist(const geometry_msgs::Twist &msg);

//msg type imports
ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::Int32 left_ticks_msg;
std_msgs::Int32 right_ticks_msg;

// pubs and subs
ros::Publisher leftEncoderTicks("/left_encoder_ticks", &left_ticks_msg);
ros::Publisher rightEncoderTicks("/right_encoder_ticks", &right_ticks_msg);
ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &handle_Twist); // keyboard input subscriber

//ROS loop rate
//ros::Rate loop_rate(10);


void setup()
{
    nh.initNode();
    nh.subscribe(sub);
    //nh.advertise(leftEncoderTicks);
    nh.advertise(rightEncoderTicks);
    nh.advertise(leftEncoderTicks);

    //define encoder pins
    pinMode(rightMotorEncoderPinA, INPUT_PULLUP); //must start with inputs Pulled up
    pinMode(rightMotorEncoderPinB, INPUT_PULLUP); //must start with inputs Pulled up

     //define encoder pins
    pinMode(leftMotorEncoderPinA, INPUT_PULLUP); //must start with inputs Pulled up
    pinMode(leftMotorEncoderPinB, INPUT_PULLUP); //must start with inputs Pulled up

    //attach interrupts
    attachInterrupt(digitalPinToInterrupt(rightMotorEncoderPinA), handleChangeA, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(rightMotorEncoderPinB), handleChangeB, CHANGE); 

    //attach interrupts
    attachInterrupt(digitalPinToInterrupt(leftMotorEncoderPinA), handleChangeC, CHANGE); 
    attachInterrupt(digitalPinToInterrupt(leftMotorEncoderPinB), handleChangeD, CHANGE); 

    //OLD define encoder pins as inputs
   pinMode (outputRightA,INPUT);
   pinMode (outputRightB,INPUT);
   pinMode (outputLeftA,INPUT);
   pinMode (outputLeftB,INPUT);

    //initialize motor shield pins as output pins
    pinMode(IN1_RIGHT, OUTPUT);
    pinMode(IN2_RIGHT, OUTPUT);
    pinMode(PWM_RIGHT, OUTPUT);
    pinMode(IN1_LEFT, OUTPUT);
    pinMode(IN2_LEFT, OUTPUT);
    pinMode(PWM_LEFT, OUTPUT);
    pinMode(STBY, OUTPUT);

    //Enable the Motor Shield output;  
    digitalWrite(6, HIGH);  
}

void loop()
{
    //readRightEncoder();
    //readLeftEncoder();
    nh.spinOnce();
    //delay(10);
}

//functions to control state of standby pin on TB6612FNG, low sets standby mode on, high removes standby
void set_standby(bool is_on)
{
    if (is_on)
    {
        digitalWrite(STBY, LOW);
    }
    else
    {
        digitalWrite(STBY, HIGH);
    }
}

//callback function to handle published messages on /cmd_vel topic
void handle_Twist(const geometry_msgs::Twist &msg)
{
    //min function: calculates the minimum of two numbers.
    float x = max(min(msg.linear.x, 1.0f), -1.0f);  // minimum value (-1) maximum value (1)
    float z = max(min(msg.angular.z, 1.0f), -1.0f); // minimum value (-1) maximum value (1)
    
    // Transforming linear and angular velocities to speed for the LEFT RIGHT MOTORS
    float l = (msg.linear.x - msg.angular.z) / 2;
    if (l > 0.0)
    {
        move_forward_left = true;
    }
    else
    {
        move_forward_left = false;
    }

    float r = (msg.linear.x + msg.angular.z) / 2;
    if (l > 0.0)
    {
        move_forward_right = true;
    }
    else
    {
        move_forward_right = false;
    }

    uint16_t lPwm = cmdvel_to_Pwm(fabs(l), PWM_MIN, PWMRANGE);
    uint16_t rPwm = cmdvel_to_Pwm(fabs(r), PWM_MIN, PWMRANGE);

    setMotorLeft(lPwm, move_forward_left);
    setMotorRight(rPwm, move_forward_right);
}

// for transforming cmd_vel float to a pwm speed
float cmdvel_to_Pwm(float x, float out_min, float out_max)
{
    return x * (out_max - out_min) + out_min;
}

void setMotorRight(int motorSpeed, bool is_forward)
{
    set_standby(false);
    if (is_forward)
    {
        digitalWrite(IN1_RIGHT, HIGH);
        digitalWrite(IN2_RIGHT, LOW);
    }
    else if (!is_forward)
    {
        digitalWrite(IN1_RIGHT, LOW);
        digitalWrite(IN2_RIGHT, HIGH);
    }
    else
    {
        digitalWrite(IN1_RIGHT, HIGH);
        digitalWrite(IN2_RIGHT, HIGH);
    }
    analogWrite(PWM_RIGHT, abs(motorSpeed));
}

void setMotorLeft(int motorSpeed, bool is_forward)
{   
    set_standby(false);
    if (is_forward)
    {
        digitalWrite(IN1_LEFT, HIGH);
        digitalWrite(IN2_LEFT, LOW);
    }
    else if (!is_forward)
    {
        digitalWrite(IN1_LEFT, LOW);
        digitalWrite(IN2_LEFT, HIGH);
    }
    else
    {
        digitalWrite(IN1_LEFT, LOW);
        digitalWrite(IN2_LEFT, LOW);
    }
    analogWrite(PWM_LEFT, abs(motorSpeed));
}

void readRightEncoder() { 

 rightAState = digitalRead(outputRightA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (rightAState != rightALastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputRightB) != rightAState) { 
       rightCounter ++;
     } else {
       rightCounter --;
     }
     Serial.print("Position: ");
     Serial.println(rightCounter);

     //publish the ticks via ROS
     ticks_msg.data = rightCounter;
    rightEncoderTicks.publish(ticks_msg.data);


   } 
   rightALastState = rightAState; // Updates the previous state of the outputA with the current state

 }


 void readLeftEncoder() { 

 leftAState = digitalRead(outputLeftA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (leftAState != leftALastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputLeftB) != leftAState) { 
       leftCounter ++;
     } else {
       leftCounter --;
     }
     Serial.print("Position: ");
     Serial.println(leftCounter);

     //publish the ticks via ROS
     ticks_msg.data = leftCounter;
    leftEncoderTicks.publish(ticks_msg.data);
   } 
   leftALastState = leftAState; // Updates the previous state of the outputA with the current state

 }

void handleChangeA() {
    if(readA == readB) {
        rightCount++;
        right_ticks_msg.data = rightCount;
        rightEncoderTicks.publish(&right_ticks_msg);
    } else {
        rightCount--;
        right_ticks_msg.data = rightCount;
        rightEncoderTicks.publish(&right_ticks_msg);
    }
}


void handleChangeB() {
    if(readA != readB) {
        rightCount++;
        //publish the ticks via ROS
        right_ticks_msg.data = rightCount;
        rightEncoderTicks.publish(&right_ticks_msg);
    } else {
        rightCount--;
        right_ticks_msg.data = rightCount;
        rightEncoderTicks.publish(&right_ticks_msg);
    }
}

void handleChangeC() {
    if(readC == readD) {
        leftCount++;
        //publish the ticks via ROS
        left_ticks_msg.data = leftCount;
        leftEncoderTicks.publish(&left_ticks_msg);
    } else {
        leftCount--;
        left_ticks_msg.data = leftCount;
        leftEncoderTicks.publish(&left_ticks_msg);
    }
}

void handleChangeD() {
    if(readC != readD) {
        leftCount++;
        //publish the ticks via ROS
        left_ticks_msg.data = leftCount;
        leftEncoderTicks.publish(&left_ticks_msg);
    } else {
        leftCount--;
        left_ticks_msg.data = leftCount;
        leftEncoderTicks.publish(&left_ticks_msg);
    }
}

