const byte AIN1 = 13;
const byte AIN2 = 12;
const byte PWMA = 11;

const byte BIN1 = 8; 
const byte BIN2 = 9; 
const byte PWMB = 10;
int STBY = 5; //standby

void setup()
{
pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(PWMA, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);
pinMode(PWMB, OUTPUT);
pinMode(STBY, OUTPUT);
}

void loop() {
      digitalWrite(STBY, HIGH); //disable standby
      //drive forward medium speed for one second
      setMotorA(100);
      setMotorB(100);
      delay(800);

      //drive backward quickly for one second
      setMotorA(100);
      setMotorB(100);
      delay(800);

      //stop for one second
      setMotorA(0);
      setMotorB(0);
      delay(800);

      //turn for one second
      setMotorA(100);
      setMotorB(-100);
      delay(2000);

}

void setMotorB(int motorSpeed)
{
  if (motorSpeed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  else if (motorSpeed < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  analogWrite(PWMB, abs(motorSpeed));
}

void setMotorA(int motorSpeed)
{
if (motorSpeed > 0)
     {
       digitalWrite(AIN1, HIGH);
       digitalWrite(AIN2, LOW);
      }
else if (motorSpeed < 0)
     {
       digitalWrite(AIN1, LOW);
       digitalWrite(AIN2, HIGH);
      }
else {
       digitalWrite(AIN1, HIGH);
       digitalWrite(AIN2, HIGH);
     }
analogWrite(PWMA, abs(motorSpeed)); 
}