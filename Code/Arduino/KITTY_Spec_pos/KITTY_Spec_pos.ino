//Here 1 is for hip joint and 2 is for knee joint//
#define motor1  6
#define motor2  4
#define motor1pwm  10
#define motor2pwm  11

//Varaibles
float w=0.1, alpha, theta1c = 0.0 , theta2c = 0.0, theta1, theta2, error1, error2, correction1, correction2;
float dif_error1 , prev_error1 = 0.0 , dif_error2 , prev_error2 = 0.0 , I=0.0, l = 48.0;
float Kp1 = 0.75, Kp2 = 0.0, Kd1 = 0.3, Kd2 = 0.0 , Ki1 = 0.0;
int l1 = 26, l2 = 22, a = 20, b = 7;

//Encoder variables
volatile int temp1 , counter1 = 0;
volatile int temp2 , counter2 = 0;

void setup() {
  Serial.begin(9600);

  //Pins for encoders
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(2, ai2, RISING);
  attachInterrupt(3, ai3, RISING);

  //Pins for Motors
  pinMode(motor1, OUTPUT);
  pinMode(motor1pwm, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor2pwm, OUTPUT);

  //Initial Conditions
//  digitalWrite(motor1, HIGH);
  
}
int flag =0;
void loop() {
if (flag == 1)
{ analogWrite(motor1pwm,0);
  Serial.println("stopped");
  }
  
 if (flag == 0){ 
  for (float t = 0; t <= 1000; t=t+0.3) {

    
//Ellipse coordinate update
//      float t = millis();
//      Serial.println(t);
//      float xe = a * cos(w*(t));
//      float ye = b * sin(w*(t)) - 40;
//      if (ye < -40) {
//    float  ye = -40;
//      }
    float xe = 48*cos(-35-w*(t) );
    float ye = 48*sin(-35-w*(t));

//Convert Encoder Output into angle
      if ( counter1 != temp1 ) {
        temp1 = counter1;
        if (counter1 > 1200) {
          counter1 = 0;
         }
        theta1c = - (counter1 * 0.3);
//        Serial.println (theta1c);
     }
      if ( counter2 != temp2 ) {
        temp2 = counter2;
        if (counter2 > 1200) {
          counter2 = 0;
        }
        theta2c = (counter2 * 0.3);
//        Serial.println (theta2c);
      }
//      

       

////Ellipse coordinates corresponding angle
      if (atan(ye / xe) > 0) {
        alpha = atan(ye / xe) - PI;
      }
      else {
        alpha = atan(ye / xe);
      }
      theta1 = 57.2958*(cosine_rule(l2, l1, sqrt((xe * xe) + (ye * ye))) + alpha);
      theta2 = 57.2958*(-PI + cosine_rule(sqrt(xe * xe + ye * ye), l1, l2));
      error1 = theta1 - theta1c;
      error2 = theta2 - theta2c;
      dif_error1 = error1 - prev_error1;
      prev_error1 = error1;
      dif_error2 = error2 - prev_error2;
      prev_error2 = error2;
      
      I += error1;
      correction1 = Kp1*error1 + Kd1*(dif_error1);//+ Ki1*(I);
      correction2 = Kp2*error2 + Kd2*(dif_error2);
//      Serial.println(error1);
//      correction1 = Kp1*error1;
//      correction2 = Kp2*error2;
      Serial.println(correction1);
//      Moving condtions
      if(theta1c < -30.0){
        flag = 1;
        break;
        }
      if(error1<0 ){
        upr_mtr_fwd();
        analogWrite(motor1pwm,abs(correction1));
        }
       else if(error1>0){
        upr_mtr_bwd();
        analogWrite(motor1pwm,abs(correction1));
        }

//      if(error2<0){
//        lwr_mtr_fwd();
//        analogWrite(motor2pwm,abs(correction2));
//        }
//      else if(error2>0){
//        lwr_mtr_bwd();
//        analogWrite(motor2pwm,abs(correction2));
//        }
//
////   float t0 = millis();
////   Serial.println(t0-t);
  }
 }
}

//Cosine Rule
float cosine_rule(float c, float b, float a) {
  float x = ( a * a + b * b - c * c ) / ( 2 * a * b );
  return acos(x);
}

//motor directions
void upr_mtr_fwd() {
  digitalWrite(motor1, HIGH);
}

void upr_mtr_bwd() {
  digitalWrite(motor1, LOW);
}

void lwr_mtr_fwd() {
  digitalWrite(motor2, HIGH);
}

void lwr_mtr_bwd() {
  digitalWrite(motor2, LOW);
}

//Encoder1
void ai0() {
  if (digitalRead(3) == LOW) {
    counter1++;
  } else {
    counter1--;
  }
}

void ai1() {
  if (digitalRead(2) == LOW) {
    counter1--;
  } else {
    counter1++;
  }
}

//  Encoder2
void ai2() {
  if (digitalRead(20) == LOW) {
    counter2++;
  } else {
    counter2--;
  }
}

void ai3() {
  if (digitalRead(21) == LOW) {
    counter2--;
  } else {
    counter2++;
  }
}
