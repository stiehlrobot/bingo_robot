/*
Code for operating 5 ultrasonic sensors on Arduino using rosserial and publishing data as type Sensor_msgs/Range. 
Sensors are numbered 1 to 5 starting from left side of robots perspective. 
*/

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range1("/ultrasonic1_range", &range_msg);
ros::Publisher pub_range2("/ultrasonic2_range", &range_msg);
ros::Publisher pub_range3("/ultrasonic3_range", &range_msg);
ros::Publisher pub_range4("/ultrasonic4_range", &range_msg);
ros::Publisher pub_range5("/ultrasonic5_range", &range_msg);

ros::Publisher pubranges[] = {pub_range1, pub_range2, pub_range3, pub_range4, pub_range5};

char frameid[] = "/ultrasound";

const int pingPin = 7;
const boolean CENTIMETERS = true;
const boolean INCHES = false;



const int ultrasonic_trigPins[] = {22, 24, 26, 28, 30};
const int ultrasonic_echoPins[] = {23, 25, 27, 29, 31};


void setup() {

    nh.initNode();
    nh.advertise(pub_range1);
    nh.advertise(pub_range2);
    nh.advertise(pub_range3);
    nh.advertise(pub_range4);
    nh.advertise(pub_range5);

    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id = frameid;
    range_msg.field_of_view = 0.1;
    range_msg.min_range = 0.002; // 2 cm
    range_msg.max_range = 0.450; // 450 cm

    Serial.begin(9600); // Starts the serial communication

}
void loop() {
    fetch_ultrasonic_reading();
    nh.spinOnce();
    delay(300);

}

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