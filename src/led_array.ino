//The code for the ledarray on the sidearray of the robot 3 different modes

//define ledpins
int left_ledPins[] = {
  11, 12, 10, 9, 8, 7
};
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
  test_move_forward();
  test_move_forward();
  test_move_forward();
  test_move_forward();
  
 
  
  
}

void circulate_leds() {

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(100);
    }

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], LOW);   // turn the LED on (HIGH is the voltage level)
        delay(100);
    }
  

}

void move_forward_led() {

    digitalWrite(12, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(11, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(10, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(9, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(8, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(7, LOW);   // turn the LED on (HIGH is the voltage level)

    delay(100);

    digitalWrite(12, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(11, HIGH);   // turn the LED on (HIGH is the voltage level)

    delay(100);
    
    digitalWrite(11, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(10, HIGH);   // turn the LED on (HIGH is the voltage level)

    delay(100);
    
    digitalWrite(10, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(9, HIGH);   // turn the LED on (HIGH is the voltage level)

    delay(100);
    
    digitalWrite(9, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(8, HIGH);   // turn the LED on (HIGH is the voltage level)

    
    delay(100);
    
    digitalWrite(8, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(7, HIGH);   // turn the LED on (HIGH is the voltage level)


}

void all_blink() {

     for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        
    }

    delay(500);

    for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], LOW);   // turn the LED on (HIGH is the voltage level)
        
    }

    delay(500);

}

void test_move_forward() {
     for (int currentPin = 0; currentPin < pinCount; currentPin++) {
        digitalWrite(left_ledPins[currentPin], HIGH);   // turn the LED on (HIGH is the voltage level)
        if(currentPin == 0) {
            digitalWrite(left_ledPins[pinCount-1], LOW);
        }
        else {
            digitalWrite(left_ledPins[currentPin-1], LOW);
        }
        delay(100);
    }

}