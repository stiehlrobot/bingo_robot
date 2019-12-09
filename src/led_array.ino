//The code for the ledarray on the sidearray of the robot 3 different modes

//define left ledArray ledpins
int left_ledPins[] = {
  11, 12, 10, 9, 8, 7
};

//define items in array
int pinCount = 6;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize the led pins in the ledpins array as outputs

  for (int currentPin = 0; currentPin < pinCount; currentPin++) {
    pinMode(left_ledPins[currentPin], OUTPUT);
  }
  
}

// the loop function runs over and over again forever
void loop() {
  
  
 
  
  
}

void circulate_leds(int intervalMillis) {

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(intervalMillis);
    }

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], LOW);   // turn the LED on (HIGH is the voltage level)
        delay(intervalMillis);
    }
  

}



void all_blink(int blinkIntervalMillis) {

     for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        
    }

    delay(blinkIntervalMillis);

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], LOW);   // turn the LED on (HIGH is the voltage level)
        
    }

    delay(blinkIntervalMillis);

}

void test_move_forward(int stepIntervalMillis) {
     for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        if(currentPin == 0) {
            digitalWrite(left_ledPins[pinCount-1], LOW);
        }
        else {
            digitalWrite(left_ledPins[currentPin-1], LOW);
        }
        delay(stepIntervalMillis);
    }

}