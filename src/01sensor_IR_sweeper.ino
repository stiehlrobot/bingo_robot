// write the code for the IR sweeper algorithm that controls servo sweep and IR interrupter
// publish the binary data from the IR interrupter to Sensor_msgs/IR



/* based on Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position


void setup() {
  myservo.attach(34);  // attaches the servo on pin 9 to the servo object
  pinMode(35,INPUT);
  Serial.begin(9600);
}

void loop() {

  for (pos = 0; pos <= 180; pos += 5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree

    // ground is where it should be
    if(digitalRead(6)==HIGH)  {
            Serial.println("Somebody is here.");
        }
        //The ground is not where it should be
        else  {
            Serial.println("Nobody.");
        }
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  
  //change direction of sweep
  for (pos = 180; pos >= 0; pos -= 5) { // goes from 180 degrees to 0 degrees

    if(digitalRead(6)==HIGH)  {
            Serial.println("Somebody is here.");
        }
        //The ground is not where it should be
        else  {
            Serial.println("Nobody.");
        }
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
}