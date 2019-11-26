// code from https://playground.arduino.cc/Code/PIDLibrary/



#include <Arduino.h>


//define the constants P, I, D as Kp, Ki, Kd
double kp;
double ki;
double kd;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

void setup() {

    setPoint = 0;
}

void loop() {
    //read the encoder input
    input = analogRead(A0);
    //provide the encoder input as input for the PID
    output = computePID(input);
    delay(100);
    //write the output as a motorspeed in PWM signal
    analogWrite(3, output);

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


