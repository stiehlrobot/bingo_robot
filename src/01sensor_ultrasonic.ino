
#include <Arduino.h>

/*
Code for operating 5 ultrasonic sensors. Sensors are numbered 1 to 5 starting from left side of robots perspective. 
Next steps is to implement as ros node on ardu:
https://github.com/surabhi96/Library-navigating-robot/wiki/Ultrasonic-sensor-with-ROS
*/


// defines trigger pins
const int ultrasonic_1_trigPin = 22;
const int ultrasonic_2_trigPin = 24;
const int ultrasonic_3_trigPin = 32;
const int ultrasonic_4_trigPin = 28;
const int ultrasonic_5_trigPin = 30;

long duration;
int distance;

int triggerPins[] = {ultrasonic_1_trigPin, ultrasonic_2_trigPin, ultrasonic_3_trigPin, ultrasonic_4_trigPin, ultrasonic_5_trigPin};

// defines echo pins
const int ultrasonic_1_echoPin = 23;
const int ultrasonic_2_echoPin = 25;
const int ultrasonic_3_echoPin = 33;
const int ultrasonic_4_echoPin = 29;
const int ultrasonic_5_echoPin = 31;

int echoPins[] = {ultrasonic_1_echoPin, ultrasonic_2_echoPin, ultrasonic_3_echoPin, ultrasonic_4_echoPin, ultrasonic_5_echoPin};


// defines duration variables
long duration1;
long duration2;
long duration3;
long duration4;
long duration5;


long durations[] = {duration1, duration2, duration3, duration4, duration5};

// defines distance variables
int distance1;
int distance2;
int distance3;
int distance4;
int distance5;

int distances[] = {distance1, distance2, distance3, distance4, distance5};

void setup() {

pinMode(ultrasonic_1_trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(ultrasonic_2_trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(ultrasonic_3_trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(ultrasonic_4_trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(ultrasonic_5_trigPin, OUTPUT); // Sets the trigPin as an Output

pinMode(ultrasonic_1_echoPin, INPUT); // Sets the echoPin as an Input
pinMode(ultrasonic_2_echoPin, INPUT); // Sets the echoPin as an Input
pinMode(ultrasonic_3_echoPin, INPUT); // Sets the echoPin as an Input
pinMode(ultrasonic_4_echoPin, INPUT); // Sets the echoPin as an Input
pinMode(ultrasonic_5_echoPin, INPUT); // Sets the echoPin as an Input


Serial.begin(9600); // Starts the serial communication

}
void loop() {
    fetch_ultrasonic_reading();

}

void fetch_ultrasonic_reading() {

    //simplified approach with arrays
    for (int i=0; i<5; i++) {

        digitalWrite(triggerPins[i], LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPins[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPins[i], LOW);
        duration = pulseIn(echoPins[i], HIGH);
        distance = duration*0.034/2;
        Serial.print("Distance for "+String(i)+" is: ");
        Serial.println(distance);
        Serial.println("-------------------------------------------------------");
        Serial.println("-------------------------------------------------------");
        Serial.println("-------------------------------------------------------");

    }

    /*
    //clear trigger pin to enhance reading
    digitalWrite(ultrasonic_1_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_2_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_3_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_4_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonic_5_trigPin, LOW);
    delayMicroseconds(2);

    // Set the trigger pin HIGH for 10 ms
    digitalWrite(ultrasonic_1_trigPin, HIGH);
    digitalWrite(ultrasonic_2_trigPin, HIGH);
    digitalWrite(ultrasonic_3_trigPin, HIGH);
    digitalWrite(ultrasonic_4_trigPin, HIGH);
    digitalWrite(ultrasonic_4_trigPin, HIGH);
    
    delayMicroseconds(10);
    
    digitalWrite(ultrasonic_1_trigPin, LOW);
    digitalWrite(ultrasonic_2_trigPin, LOW);
    digitalWrite(ultrasonic_3_trigPin, LOW);
    digitalWrite(ultrasonic_4_trigPin, LOW);
    digitalWrite(ultrasonic_4_trigPin, LOW);

    // Return the duration of the sound wave in microseconds
    duration1 = pulseIn(ultrasonic_1_echoPin, HIGH);
    duration2 = pulseIn(ultrasonic_2_echoPin, HIGH);
    duration3 = pulseIn(ultrasonic_3_echoPin, HIGH);
    duration4 = pulseIn(ultrasonic_4_echoPin, HIGH);
    duration5 = pulseIn(ultrasonic_5_echoPin, HIGH);

    // Calculate the distance
    distance1 = duration1*0.034/2;
    distance2 = duration2*0.034/2;
    distance3 = duration3*0.034/2;
    distance4 = duration4*0.034/2;
    distance5 = duration5*0.034/2;

    // Prints the distance on the Serial Monitor
    Serial.print("Distance1: ");
    Serial.println(distance1);
    Serial.print("Distance2: ");
    Serial.println(distance2);
    Serial.print("Distance3: ");
    Serial.println(distance3);
    Serial.print("Distance4: ");
    Serial.println(distance4);
    Serial.print("Distance5: ");
    Serial.println(distance5);
    */

}