 
 #include <Arduino.h>


 #define outputRightA 40
 #define outputRightB 41
 #define outputLeftA 42
 #define outputLeftB 43


//define the constants P, I, D as Kp, Ki, Kd
double kp = 2;
double ki = 5;
double kd = 1;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;



 int leftCounter = 0;
 int rightCounter = 0;
 int rightAState;
 int leftAState;
 int rightALastState;  
 int leftALastState;

 void setup() { 
    // Motor_1 control pin initiate;
    pinMode(4, OUTPUT);     
    pinMode(5, OUTPUT);    
    pinMode(9, OUTPUT); // Speed control
    
    // Motor_2 control pin initiate;
    pinMode(7, OUTPUT);     
    pinMode(8, OUTPUT);    
    pinMode(10, OUTPUT);  // Speed control
    
    //Enable the Motor Shield output;  
    pinMode(6, OUTPUT); 
    digitalWrite(6, HIGH);  



   pinMode (outputRightA,INPUT);
   pinMode (outputRightB,INPUT);
   pinMode (outputLeftA,INPUT);
   pinMode (outputLeftB,INPUT);
   
   Serial.begin (9600);
   // Reads the initial state of the outputA
   rightALastState = digitalRead(outputRightA);   
   leftALastState = digitalRead(outputLeftA);  


    //define target position
   setPoint = 500;
 } 

 void loop() { 
    readRightEncoder();
   //read the encoder input
    input = rightCounter;
    //provide the encoder input as input for the PID
    output = computePID(input);
    delay(100);
    //write the output as a motorspeed in PWM signal
    Serial.println("PID output is:");
    Serial.println(output);
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
     // Serial.print("Position: ");
     //Serial.println(rightCounter);
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
     //Serial.print("Position: ");
     //Serial.println(leftCounter);
   } 
   leftALastState = leftAState; // Updates the previous state of the outputA with the current state

 }


double computePID(double inp){

    // determine the passage of time
    currentTime = millis();
    elapsedTime = currentTime - previousTime;

    //determine the error between setpoint and encoder input
    error = setPoint - inp;

    
    //compute the integral, the cumulative error over time
    cumError += error * elapsedTime;

    //compute the derivative, the rate of change
    rateError = (error - lastError)/elapsedTime;

    //Kp, Ki and Kd are predetermined constants. 
    double output = Kp * error + Ki * cumError + Kd * rateError;

    //assign the error of this iteration as the lastError for the next run of the script
    lastError = error;

    //assign the previoustime variable to currentTime as the loop is finished
    previousTime = currentTime;

    return out;
}
