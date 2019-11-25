
//original source code at https://forum.arduino.cc/index.php?topic=488275.0


const byte encoderPinA = 40;//outputA digital pin2
const byte encoderPinB = 41;//outoutB digital pin3
volatile int count = 0;
int protectedCount = 0;
int previousCount = 0;

#define readA bitRead(PIND,2)//faster than digitalRead()
#define readB bitRead(PIND,3)//faster than digitalRead()


void setup() {
  Serial.begin (115200);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(encoderPinA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), isrB, CHANGE);
}

void loop() {
  noInterrupts();
  protectedCount = count;
  interrupts();
 
  if(protectedCount != previousCount) {
    Serial.println(protectedCount);
  }
  previousCount = protectedCount;
}

void isrA() {
  if(readB != readA) {
    count ++;
  } else {
    count --;
  }
}
void isrB() {
  if (readA == readB) {
    count ++;
  } else {
    count --;
  }
}
