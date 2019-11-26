

//available pins for arduino mega for interrupts are 2,3,21,20,19,18 and as a Arduino Mega specific case 
//must be denoted using numbers 0, 1, 2, 3, 4, 5, 6 when using attachInterrupt(pin)

//basic logic is this: encoder has two channels which oscillate between HIGH and LOW, but they are not identical patterns
//We use arduinos built in interrupt mechanism to monitor state changes in channel A
//There are 4 different scenarios when channel A state changes from high to low or vice versa.

//WHEEL DIRECTION 1 FOR THESE CASES:
//CASE1: When A is HIGH and B is HIGH
//CASE2: When A is LOW and B is LOW

//WHEEL DIRECTION 2 FOR THESE CASES:
//CASE3: When A is HIGH and B is LOW
//CASE4: When A is LOW and B is HIGH

#include <Arduino.h>

const byte rightMotorEncoderPinA = 21; // Arduino mega pin 20 for encoder output A
const byte rightMotorEncoderPinB = 20; // Arduino mega pin 21 for encoder output B

volatile long rightCount = 0; //arduino reference recommends using volatile type when working with counters for interrupts

int protectedCount = 0;
int previousCount = 0;

#define readA digitalRead(rightMotorEncoderPinA)
#define readB digitalRead(rightMotorEncoderPinB)

//#define readA bitRead(PIND,21) //faster than digitalRead()
//#define readB bitRead(PIND,20) //faster than digitalRead()



void setup() {
    Serial.begin(9600);

    pinMode(rightMotorEncoderPinA, INPUT_PULLUP); //must start with inputs Pulled up
    pinMode(rightMotorEncoderPinB, INPUT_PULLUP); //must start with inputs Pulled up

    attachInterrupt(digitalPinToInterrupt(rightMotorEncoderPinA), handleChangeA, CHANGE); //so without using digitalPinToInterrupt I assign the pin directly to 3 which is pin 20 on the arduino mega
    attachInterrupt(digitalPinToInterrupt(rightMotorEncoderPinB), handleChangeB, CHANGE); //so without using digitalPinToInterrupt I assign the pin directly to 2 which is pin 21 on the arduino mega
}

void loop() {
  //noInterrupts();
  //protectedCount = rightCount;
  //interrupts();
 
  //if(protectedCount != previousCount) {
  //  Serial.println(protectedCount);
  //}
  //previousCount = protectedCount;
  ///Serial.println(rightCount);
}

void handleChangeB() {
    if(readA != readB) {
        rightCount++;
        Serial.println(rightCount);
    } else {
        rightCount--;
        Serial.println(rightCount);
    }
}

void handleChangeA() {
    if(readA == readB) {
        rightCount++;
        Serial.println(rightCount);
    } else {
        rightCount--;
        Serial.println(rightCount);
    }
}



