#include <Arduino.h>

#define PWMRANGE 255
#define PWM_MIN 0

const byte IN1_LEFT = 13;
const byte IN2_LEFT = 12;
const byte PWM_LEFT = 11;

const byte IN1_RIGHT = 8;
const byte IN2_RIGHT = 9;
const byte PWM_RIGHT = 10;
int STBY = 5; //standby
bool move_forward_left = true;
bool move_forward_right = true;

void setup()
{
    Serial.begin(9600);
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

    Serial.println("Starting motor test script");
    delay(1000);
    Serial.println("Turned of standby mode");
    delay(1000);

    //run left motor backward for 3 seconds
    left_motor_backward(200);
    Serial.println("left motor backward for 3 seconds");
    delay(3000);
    motor_brake("left");
    Serial.println("motor brake applied on LEFT motor");
    delay(1000);

    //run left motor forward for 3 second
    left_motor_forward(200);
    Serial.println("left motor forward for 3 seconds");
    delay(3000);
    motor_brake("left");
    Serial.println("motor brake applied on LEFT motor");
    delay(1000);
    

    //run right motor backward for 3 seconds
    right_motor_forward(200);
    Serial.println("right motor forward for 3 seconds");
    delay(3000);
    motor_brake("right");
    Serial.println("motor brake applied on RIGHT motor");
    delay(1000);

    //run right motor forward for 3 seconds
    right_motor_backward(200);
    Serial.println("right motor backward for 3 seconds");
    delay(3000);
    motor_brake("right");
    Serial.println("motor brake applied on RIGHT motor");
    delay(1000);

    //end motor test and set standby mode
    set_standby(true);
    Serial.println("Turned on standby mode");
    Serial.println("------------END-----------");
}

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

void left_motor_forward(int pwm_speed)
{

    set_standby(false);
    digitalWrite(IN1_LEFT, HIGH);
    digitalWrite(IN2_LEFT, LOW);
    analogWrite(PWM_LEFT, pwm_speed);
}

void left_motor_backward(int pwm_speed)
{

    set_standby(false);
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, HIGH);
    analogWrite(PWM_LEFT, pwm_speed);
}

void right_motor_forward(int pwm_speed)
{

    set_standby(false);
    digitalWrite(IN1_RIGHT, HIGH);
    digitalWrite(IN2_RIGHT, LOW);
    analogWrite(PWM_RIGHT, pwm_speed);
}

void right_motor_backward(int pwm_speed)
{

    set_standby(false);
    digitalWrite(IN1_RIGHT, LOW);
    digitalWrite(IN2_RIGHT, HIGH);
    analogWrite(PWM_RIGHT, pwm_speed);
}



void motor_brake(String motor_name)
{
    if (motor_name == "left")
    {
        //stop left motor
        digitalWrite(IN1_LEFT, HIGH);
        digitalWrite(IN2_LEFT, HIGH);
    }
    else if (motor_name == "right")
    {
        //stop right motor
        digitalWrite(IN1_RIGHT, HIGH);
        digitalWrite(IN2_RIGHT, HIGH);
    }
    else
    {

        Serial.print("no motor of this name recognized.");
    }
}

void rotate_robot(bool is_clockwise, int time_in_millis) {

    //add if else to theck if clockwise or counter clockwise

    t0 = millis()
    while((t1 - t0) < time_in_millis):
        t1 = millis()

       
    //run left motor forward for 3 second
        left_motor_forward(200);
        
        
        
        delay(1000);

    motor_brake("left")
    Serial.println("motor brake applied on LEFT motor");
    Serial.printl("Finished rotating")
    delay(1000);



}