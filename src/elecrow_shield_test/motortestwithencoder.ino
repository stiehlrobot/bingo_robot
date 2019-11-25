
//testing use of DC motors with the elecrow 2 channel H-bridge
#include <Arduino.h>
// pins for the encoder inputs
#define RH_ENCODER_A 40 
#define RH_ENCODER_B 41
#define LH_ENCODER_A 42
#define LH_ENCODER_B 43

// variables to store the number of encoder pulses
// for each motor
volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;

void setup() {                

 pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  
  // initialize hardware interrupts
  attachInterrupt(0, leftEncoderEvent, CHANGE);
  attachInterrupt(1, rightEncoderEvent, CHANGE);
 // Motor_1 controll pin initiate;
 pinMode(4, OUTPUT);     
 pinMode(5, OUTPUT);    
 pinMode(9, OUTPUT); // Speed control
 
 // Motor_2 controll pin initiate;
 pinMode(7, OUTPUT);     
 pinMode(8, OUTPUT);    
 pinMode(10, OUTPUT);  // Speed control
 
 //Enable the Motor Shield output;  
 pinMode(6, OUTPUT); 
 digitalWrite(6, HIGH);  
}
void loop() {
    Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  
   analogWrite(9,25);    // set the motor_1 speed ;
   digitalWrite(4, LOW);   
   digitalWrite(5, HIGH);  // Set the rotation of motor_1
   
   analogWrite(10,25);    // set the motor_2 speed ;
   digitalWrite(7, HIGH);  
   digitalWrite(8, LOW);  // Set the rotation of motor_1
 
  delay(1000);               // wait for a 5 seconds
  Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  delay(1000);               // wait for a 5 seconds
  Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  // And we change the motor speed and  rotation direction
    analogWrite(9, 50);    // set the motor_1 speed to 100 ;
   digitalWrite(4, HIGH);   
   digitalWrite(5, LOW);  // Set the rotation of motor_1
   
   analogWrite(10,50);    // set the motor_2 speed to 150
   digitalWrite(7, LOW);  
   digitalWrite(8, HIGH);  // Set the rotation of motor_1
   delay(1000);               // wait for a 5 seconds
   Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  delay(1000);               // wait for a 5 seconds
   Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  delay(1000);               // wait for a 5 seconds
   Serial.print("Right Count: ");
  Serial.println(rightCount);
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
}

// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}

// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}