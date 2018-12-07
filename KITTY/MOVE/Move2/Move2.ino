#define motor1  28
#define motor2  29
#define motor1pwm  5
#define motor2pwm  4

float theta1c=0, theta2c=0;
int x;

volatile int temp1, counter1 = 0;
volatile int temp2 , counter2 = 0;
void setup() {
  Serial.begin(9600);
  //Pins for encoders
  pinMode(19, INPUT_PULLUP);                                           
  pinMode(9, INPUT_PULLUP);
  attachInterrupt(4, ai4_2, RISING);

  
  pinMode(18, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(5, ai5_2, RISING);

  //Pins for Motors
  pinMode(motor1, OUTPUT);
  pinMode(motor1pwm, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor2pwm, OUTPUT);

}

void loop() {
  if (Serial.available())
  {
    x = Serial.parseInt();
  }
    if (x) {
      if (x < 3)
      {
        digitalWrite(motor1, x - 1 );
        if(x==1)
        analogWrite(motor1pwm , 60);
        else
        analogWrite(motor1pwm , 50);
      }
      if(x == 3 || (-theta1c+26)>47.8){
        analogWrite(motor1pwm , 0);
        x=3;
        }
      if(x>3 && x<6){
        digitalWrite(motor2, x - 4 );
        analogWrite(motor2pwm , 50);
        }
      if(x == 6 || (-theta2c/4)>72.97){
        analogWrite(motor2pwm , 0);
        x=6;
        }
    }
  

  //Convert Encoder Output into angle
      if ( counter1 != temp1 ) {
        temp1 = counter1;
        if (counter1 > 600) {
          counter1 = 0;
         }
        theta1c = (counter1 * 0.6);
//        Serial.println("theta1c");
        Serial.println(theta1c);
     }
      if ( counter2 != temp2 ) {
        temp2 = counter2;
        if (counter2 > 600) {
          counter2 = 0;
        }
        theta2c = -(counter2 * 0.6);
//        Serial.println("theta2c");
        Serial.println(theta2c/4);
      }
}

// Encoder1
void ai4_2() {
  if (digitalRead(9) == LOW) {
    counter1++;
  } else {
    counter1--;
  }
}

////  Encoder2
void ai5_2() {
  if (digitalRead(8) == LOW) {
    counter2++;
  } else {
    counter2--;
  }
}
