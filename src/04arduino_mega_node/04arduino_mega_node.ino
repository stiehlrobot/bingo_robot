#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

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


//LED ARRAY DEFINITIONS

//define left led array ledpins
int left_ledPins[] = {
  11, 12, 10, 9, 8, 7
};

//define items in left led array
int pinCount = 6;

//define left ledArray ledpins
int right_ledPins[] = {
  11, 12, 10, 9, 8, 7
};

//ULTRASONIC ARRAY DEFINITIONS
const int ultrasonic_trigPins[] = {22, 24, 26, 28, 30};
const int ultrasonic_echoPins[] = {23, 25, 27, 29, 31};

//MOTOR SHIELD DEFINITIONS

//define motor PINS used by Elecrow 2-Channel H-bridge Arduino Shield 

// Motor_1 control pin initiate;
const byte IN1_RIGHT = 4;
const byte IN2_RIGHT = 5;
const byte PWM_RIGHT = 9;

// Motor_2 control pin initiate;
const byte IN1_LEFT = 7;
const byte IN2_LEFT = 8;
const byte PWM_LEFT = 10;

//Standby Pin
int STBY = 6; //standby

bool motor_state = false;

bool move_forward_left = true;
bool move_forward_right = true;


//functions definitions
void handle_Twist(const geometry_msgs::Twist &msg);

//msg type imports
ros::NodeHandle nh;

std_msgs::String str_msg;
std_msgs::Int32 ticks_msg;
//std_srvs::Empty;

std_msgs::Int32 left_ticks_msg;
std_msgs::Int32 right_ticks_msg;

//std_msgs::Bool motor_state_msg;

sensor_msgs::Range range_msg;



// ROS PUBLISHERS AND SUBSCRIBERS
//left and right encoder publishers
ros::Publisher leftEncoderTicks("/left_encoder_ticks", &left_ticks_msg);
ros::Publisher rightEncoderTicks("/right_encoder_ticks", &right_ticks_msg);

//ros::ServiceServer<Bool::Request, Empty::Response> enable_motor_server("/enable_motors",&enable_motor_callback);

//INCOMPLETE
//ros::Publisher motorState("/motor_state", &motor_state_msg);

//Ultrasonic Sensor publishers
ros::Publisher pub_range1("/ultrasonic1_range", &range_msg);
ros::Publisher pub_range2("/ultrasonic2_range", &range_msg);
ros::Publisher pub_range3("/ultrasonic3_range", &range_msg);
ros::Publisher pub_range4("/ultrasonic4_range", &range_msg);
ros::Publisher pub_range5("/ultrasonic5_range", &range_msg);
ros::Publisher pubranges[] = {pub_range1, pub_range2, pub_range3, pub_range4, pub_range5};

char frameid[] = "/ultrasound";



ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &handle_Twist); // keyboard input subscriber

//ROS loop rate
//ros::Rate loop_rate(10);


void setup()
{
    nh.initNode();
    nh.subscribe(sub);

    //ADVERTISE ROS PUBLISHERS TO MASTER
    
    nh.advertise(rightEncoderTicks);
    nh.advertise(leftEncoderTicks);
    //nh.advertise(motorState);

    nh.advertise(pub_range1);
    nh.advertise(pub_range2);
    nh.advertise(pub_range3);
    nh.advertise(pub_range4);
    nh.advertise(pub_range5);

    //ADVERTISE ROS SERVICES

   // nh.advertiseService(enable_motor_server);

    //Define the specifcations for the ultrasonic range msg: frame id, field of view, min and max ranges in metres
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id = frameid;
    range_msg.field_of_view = 0.1;
    range_msg.min_range = 0.002; // 2 cm
    range_msg.max_range = 0.450; // 450 cm



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


    // initialize the led pins in the ledpins array as outputs

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        pinMode(left_ledPins[currentPin], OUTPUT);
    }

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
    publishRightEncoderTicks();
    publishLeftEncoderTicks();
    nh.spinOnce();
    delay(10);
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
    if (r > 0.0)
    {
        move_forward_right = true;
    }
    else
    {
        move_forward_right = false;
    }

    uint16_t lPwm = cmdvel_to_Pwm(fabs(l), PWM_MIN, (PWMRANGE/4));
    uint16_t rPwm = cmdvel_to_Pwm(fabs(r), PWM_MIN, (PWMRANGE/4));

    setMotorLeft(lPwm, move_forward_left);
    setMotorRight(rPwm, move_forward_right);
}

// for transforming cmd_vel float to a pwm speed
float cmdvel_to_Pwm(float x, float out_min, float out_max)
{
    return x * (out_max - out_min) + out_min;
}

//handle enable motors service request

//void enable_motor_callback(const Bool::Request & req, Empty::Response & res) {
  //  request = req.data;
    //motor_state = request;
    //handle enable motors service


//}

void setMotorRight(int motorSpeed, bool is_forward)
{
    
    if (is_forward)
    {   
        //rotates clockwise
        digitalWrite(IN1_RIGHT, LOW);
        digitalWrite(IN2_RIGHT, HIGH);
       
    }
    else {   
         //rotates counter-clockwise
        digitalWrite(IN1_RIGHT, HIGH);
        digitalWrite(IN2_RIGHT, LOW);
        
    }    
    analogWrite(PWM_RIGHT, abs(motorSpeed));
}

void setMotorLeft(int motorSpeed, bool is_forward)
{   
    
    if (is_forward)
    {   
        //rotates counter-clockwise
        digitalWrite(IN1_LEFT, HIGH);
        digitalWrite(IN2_LEFT, LOW);
    }
    else {   
        //rotates clockwise
        digitalWrite(IN1_LEFT, LOW);
        digitalWrite(IN2_LEFT, HIGH);
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

//MOTOR ENCODER INTERRUPT HANDLERS BELOW

void handleChangeA() {
    if(readA == readB) {
        rightCount--;
        //right_ticks_msg.data = rightCount;
        //rightEncoderTicks.publish(&right_ticks_msg);
    } else {
        rightCount++;
        //right_ticks_msg.data = rightCount;
        //rightEncoderTicks.publish(&right_ticks_msg);
    }
}


void handleChangeB() {
    if(readA != readB) {
        rightCount--;
        //publish the ticks via ROS
        //right_ticks_msg.data = rightCount;
        //rightEncoderTicks.publish(&right_ticks_msg);
    } else {
        rightCount++;
        //right_ticks_msg.data = rightCount;
        //rightEncoderTicks.publish(&right_ticks_msg);
    }
}

void handleChangeC() {
    if(readC == readD) {
        leftCount++;
        //publish the ticks via ROS
        //left_ticks_msg.data = leftCount;
        //leftEncoderTicks.publish(&left_ticks_msg);
    } else {
        leftCount--;
        //left_ticks_msg.data = leftCount;
        //leftEncoderTicks.publish(&left_ticks_msg);
    }
}

void handleChangeD() {
    if(readC != readD) {
        leftCount++;
        //publish the ticks via ROS
        //left_ticks_msg.data = leftCount;
        //leftEncoderTicks.publish(&left_ticks_msg);
    } else {
        leftCount--;
        //left_ticks_msg.data = leftCount;
        //leftEncoderTicks.publish(&left_ticks_msg);
    }
}

void publishLeftEncoderTicks() {

    left_ticks_msg.data = -(leftCount);
    leftEncoderTicks.publish(&left_ticks_msg);
}

void publishRightEncoderTicks() {

    right_ticks_msg.data = -(rightCount);
    rightEncoderTicks.publish(&right_ticks_msg);

}

//LED ARRAY FUNCTIONS BELOW

void circulate_blink(int lightArray[], int intervalMillis) {

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(lightArray[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(intervalMillis);
    }

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(lightArray[currentPin], LOW);   // turn the LED on (HIGH is the voltage level)
        delay(intervalMillis);
    }
  

}



void warning_blink(int ledLightArray[], int blinkIntervalMillis) {

     for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(ledLightArray[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        
    }

    delay(blinkIntervalMillis);

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(ledLightArray[currentPin], LOW);   // turn the LED on (HIGH is the voltage level)
        
    }

    delay(blinkIntervalMillis);

}

void movement_blink(int ledArray[], int stepIntervalMillis) {
     for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(ledArray[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        if(currentPin == 0) {
            digitalWrite(ledArray[pinCount-1], LOW);
        }
        else {
            digitalWrite(ledArray[currentPin-1], LOW);
        }
        delay(stepIntervalMillis);
    }

}


//ULTRASONIC ARRAY FUNCTIONS


void fetch_ultrasonic_reading() {

    long duration;
    int distance;

    //simplified approach with arrays
    for (int i=0; i<5; i++) {
        
       
        pinMode(ultrasonic_trigPins[i], OUTPUT); // Sets the trigPin as an Output
        pinMode(ultrasonic_echoPins[i], INPUT); // Sets the echoPin as an Input //
        digitalWrite(ultrasonic_trigPins[i], LOW);
        delayMicroseconds(2);
        digitalWrite(ultrasonic_trigPins[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(ultrasonic_trigPins[i], LOW);
        duration = pulseIn(ultrasonic_echoPins[i], HIGH);
        distance = duration*0.034/2;
        range_msg.range = distance;
        range_msg.header.stamp = nh.now();
        pubranges[i].publish(&range_msg);
        

    }


}

void publishMotorState() {

//        motor_state_msg.data = motor_state;
  //      motorState.publish(&motor_state_msg);


}


