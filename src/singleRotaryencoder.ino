/*     Arduino Rotary Encoder Tutorial
 *      
 *  based on code by Dejan Nedelkovski, www.HowToMechatronics.com
 *  
 */
 
 #define outputRightA 40
 #define outputRightB 41
 #define outputLeftA 40
 #define outputLeftB 40


 int leftCounter = 0;
 int rightCounter = 0;
 int rightAState;
 int leftAState;
 int rightALastState;  
 int leftALastState;

 void setup() { 
   pinMode (outputRightA,INPUT);
   pinMode (outputRightB,INPUT);
   pinMode (outputLeftA,INPUT);
   pinMode (outputLeftB,INPUT);
   
   Serial.begin (9600);
   // Reads the initial state of the outputA
   rightALastState = digitalRead(outputRightA);   
   leftALastState = digitalRead(outputLeftA);  
 } 

 void loop() { 
   readRightEncoder();
   readLeftEncoder();
 }

 void readRightEncoder() { 

 rightaState = digitalRead(outputRightA); // Reads the "current" state of the outputA
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
   } 
   leftALastState = leftAState; // Updates the previous state of the outputA with the current state

 }