#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <Arduino.h>

#define PWMRANGE 255
#define PWM_MIN 0

const byte IN1_RIGHT = 13;
const byte IN2_RIGHT = 12;
const byte PWM_RIGHT = 11;

const byte IN1_LEFT = 8;
const byte IN2_LEFT = 9;
const byte PWM_LEFT = 10;
int STBY = 5; //standby
bool move_forward_left = true;
bool move_forward_right = true;

void handle_Twist(const geometry_msgs::Twist &msg);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", &handle_Twist); // keyboard input subscriber

void setup()
{
    nh.initNode();
    nh.subscribe(sub);
    pinMode(IN1_RIGHT, OUTPUT);
    pinMode(IN2_RIGHT, OUTPUT);
    pinMode(PWM_RIGHT, OUTPUT);
    pinMode(IN1_LEFT, OUTPUT);
    pinMode(IN2_LEFT, OUTPUT);
    pinMode(PWM_LEFT, OUTPUT);
    pinMode(STBY, OUTPUT);
}

void loop()
{

    nh.spinOnce();
    delay(10);
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
