#define motor1_1  27          //                                                      1-hip        2-knee
#define motor2_1  26
#define motor1pwm_1  3
#define motor2pwm_1  2

#define motor1_2  28
#define motor2_2  29
#define motor1pwm_2  5
#define motor2pwm_2  4

#include <SoftwareSerial.h>

SoftwareSerial mySerial1(10, 11); // RX, TX          

int h=1,aa=2;

double alpha_1;
float w = 0.1, theta1c_1 = 0.0 , theta2c_1 = 0.0, theta1_1, theta2_1, error1_1, error2_1, correction1_1, correction2_1, c1_1, c2_1;
float dif_error1_1 , prev_error1_1 = 0.0 , dif_error2_1 , prev_error2_1 = 0.0;

double alpha_2;
float theta1c_2 = 0.0 , theta2c_2 = 0.0, theta1_2, theta2_2, error1_2, error2_2, correction1_2, correction2_2, c1_2, c2_2;
float dif_error1_2 , prev_error1_2 = 0.0 , dif_error2_2 , prev_error2_2 = 0.0;

float Kp1 = 1.5, Kp2 = 1.5, Kd1 = 2.0, Kd2 = 2.0 ;

int l1 = 25, l2 = 25, a = 20, b = 7;

volatile int temp1_1 , counter1_1 = 0;
volatile int temp2_1 , counter2_1 = 0;

volatile int temp1_2 , counter1_2 = 0;
volatile int temp2_2 , counter2_2 = 0;


void setup()
{
  Serial.begin(9600);
  mySerial1.begin(9600);

//Leg1
  pinMode(21, INPUT_PULLUP);                                           
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(2, ai2_1, RISING);

  
  pinMode(20, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(3, ai3_1, RISING);

//  Leg2
  pinMode(19, INPUT_PULLUP);                                           
  pinMode(9, INPUT_PULLUP);
  attachInterrupt(4, ai4_2, RISING);

  
  pinMode(18, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(5, ai5_2, RISING);

  pinMode(motor1_1, OUTPUT);
  pinMode(motor1pwm_1, OUTPUT);
  pinMode(motor2_1, OUTPUT);
  pinMode(motor2pwm_1, OUTPUT);

    pinMode(motor1_2, OUTPUT);
  pinMode(motor1pwm_2, OUTPUT);
  pinMode(motor2_2, OUTPUT);
  pinMode(motor2pwm_2, OUTPUT);

}

void loop()
{
        for (float t = 0, u=0;t < 3.14159,u < 8.2; t = t + 0.3,u = u + 0.8)
        {

      float xe_1 = 12 * cos(t);
      float ye_1 = -40 + 8*sin(t);

      float xe_2 = 4 + 2.5*u ;
      float ye_2 = -40 ;

      if ( counter1_1 != temp1_1 ){
        temp1_1 = counter1_1;

        if (counter1_1 > 600)
        {
          counter1_1 = 0;
        }
        theta1c_1 = (counter1_1 * 0.6);
      }

      if ( counter2_1 != temp2_1 ){
        temp2_1 = counter2_1;

        if (counter2_1 > 600)
        {
          counter2_1 = 0;
        }
        theta2c_1 = (counter2_1 * 0.6);
      }

      if ( counter1_2 != temp1_2 ){
        temp1_2 = counter1_2;

        if (counter1_2 > 600){
          counter1_2 = 0;
          }

        theta1c_2 = (counter1_2 * 0.6);
      }

      if ( counter2_2 != temp2_2 ){
        temp2_2 = counter2_2;

        if (counter2_2 > 600)
        {
          counter2_2 = 0;
        }
        theta2c_2 = -(counter2_2 * 0.6);
      }


     
        if (atan(ye_1/xe_1)>0)
        {
          alpha_1 = atan(ye_1 / xe_1) - 3.14159;
        }
        else
        {
          alpha_1 = atan(ye_1 / xe_1);
        }

        if (atan(ye_2 / xe_2) > 0)
        {
          alpha_2 = atan(ye_2 / xe_2) - 3.14159;
        }
        else
         { alpha_2 = atan(ye_2 / xe_2);
         }
    
        theta1_1 = 57.2958 * (cosine_rule(l1,l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
        theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

        error1_1 = theta1_1 - theta1c_1 + 39.94;
        error2_1 = theta2_1 - theta2c_1/4 + 66.72;

        dif_error1_1 = error1_1 - prev_error1_1;
        dif_error2_1 = error2_1 - prev_error2_1;
       
        c1_1 = Kp1 * error1_1 + Kd1 * (dif_error1_1);
        c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

        prev_error1_1 = error1_1;
        prev_error2_1 = error2_1;

        correction1_1 = map(abs(c1_1), 0, 30, 0, 40);
        correction2_1 = map(abs(c2_1), 0, 40, 0, 60);

        Serial.print("x=");
        Serial.println(xe_1);       
        Serial.print("y=");
        Serial.println(ye_1);
        Serial.print("theta1_1=");
        Serial.println(theta1_1);
        Serial.print("theta1c_1=");
        Serial.println(theta1c_1-39.94);
        Serial.print("theta2c_1=");
        Serial.println(theta2c_1-66.72);        
        Serial.print("theta2_1=");
        Serial.println(theta2_1);
        Serial.print("c1_1=");
        Serial.println(c1_1);
        Serial.print("c2_1=");
        Serial.println(c2_1);
        Serial.print("pwm1=");
        Serial.println(correction1_1);
        Serial.print("pwm2=");
        Serial.println(correction2_1);
        Serial.println("------------------------");

        theta1_2 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_2 * xe_2) + (ye_2 * ye_2))) + alpha_2);
        theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

        error1_2 = theta1_2 - theta1c_2 + 47.8;         
        error2_2 = theta2_2 - theta2c_2/4 + 72.97;

        dif_error1_2 = error1_2 - prev_error1_2;
        dif_error2_2 = error2_2 - prev_error2_2;

        c1_2 = Kp1 * error1_2 + Kd1 * (dif_error1_2);
        c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

        prev_error1_2 = error1_2;
        prev_error2_2 = error2_2;

        correction1_2 = map(abs(c1_2), 0, 30, 0, 50);
        correction2_2 = map(abs(c2_2), 0, 40, 0, 50);

 
        if (error1_1 < 0 ){
          upr_mtr_fwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }
        
        else if (error1_1 > 0){
          upr_mtr_bwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }

        if (error2_1 < 0){
          lwr_mtr_fwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }
        
        else if (error2_1 > 0){
          lwr_mtr_bwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }

        if (error1_2 < 0 ){
          upr_mtr_fwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }
        
        else if (error1_2 > 0){
          upr_mtr_bwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }

        if (error2_2 < 0){
          lwr_mtr_fwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
        
        else if (error2_2 > 0){
          lwr_mtr_bwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
     
    }

        for (float t = 0, u=0;t < 3.14159,u < 8.2; t = t + 0.3,u = u + 0.8)
        {

      float xe_2 = 12 * cos(t);
      float ye_2 = -40 + 8 * sin(t);

      float xe_1 = -12 + 2.5*u ;
      float ye_1 = -40 ;

      if ( counter1_1 != temp1_1 ){
        temp1_1 = counter1_1;

        if (counter1_1 > 600)
        {
          counter1_1 = 0;
        }
        theta1c_1 = (counter1_1 * 0.6);
      }

      if ( counter2_1 != temp2_1 ){
        temp2_1 = counter2_1;

        if (counter2_1 > 600)
        {
          counter2_1 = 0;
        }
        theta2c_1 = (counter2_1 * 0.6);
      }

      if ( counter1_2 != temp1_2 ){
        temp1_2 = counter1_2;

        if (counter1_2 > 600){
          counter1_2 = 0;
          }

        theta1c_2 = (counter1_2 * 0.6);
      }

      if ( counter2_2 != temp2_2 ){
        temp2_2 = counter2_2;

        if (counter2_2 > 600)
        {
          counter2_2 = 0;
        }
        theta2c_2 = -(counter2_2 * 0.6);
      }


     
        if (atan(ye_1/xe_1)>0)
        {
          alpha_1 = atan(ye_1 / xe_1) - 3.14159;
        }
        else
        {
          alpha_1 = atan(ye_1 / xe_1);
        }

        if (atan(ye_2 / xe_2) > 0)
        {
          alpha_2 = atan(ye_2 / xe_2) - 3.14159;
        }
        else
         { alpha_2 = atan(ye_2 / xe_2);
         }
    
        theta1_1 = 57.2958 * (cosine_rule(l1,l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
        theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

        error1_1 = theta1_1 - theta1c_1 + 39.94;            
        error2_1 = theta2_1 - theta2c_1/4 + 66.72;

        dif_error1_1 = error1_1 - prev_error1_1;
        dif_error2_1 = error2_1 - prev_error2_1;
       
        c1_1 = Kp1 * error1_1 + Kd1 * (dif_error1_1);
        c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

        prev_error1_1 = error1_1;
        prev_error2_1 = error2_1;

        correction1_1 = map(abs(c1_1), 0, 30, 0, 50);
        correction2_1 = map(abs(c2_1), 0, 40, 0, 50);

        Serial.print("x=");
        Serial.println(xe_1);       
        Serial.print("y=");
        Serial.println(ye_1);
        Serial.print("theta1_1=");
        Serial.println(theta1_1);
        Serial.print("theta1c_1=");
        Serial.println(theta1c_1-39.94);
        Serial.print("theta2c_1=");
        Serial.println(theta2c_1-66.72);        
        Serial.print("theta2_1=");
        Serial.println(theta2_1);
        Serial.print("c1_1=");
        Serial.println(c1_1);
        Serial.print("c2_1=");
        Serial.println(c2_1);
        Serial.print("pwm1=");
        Serial.println(correction1_1);
        Serial.print("pwm2=");
        Serial.println(correction2_1);
        Serial.println("------------------------");

        theta1_2 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_2 * xe_2) + (ye_2 * ye_2))) + alpha_2);
        theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

        error1_2 = theta1_2 - theta1c_2 + 47.8;         // Need to change acc to start position
        error2_2 = theta2_2 - theta2c_2/4 + 72.97;

        dif_error1_2 = error1_2 - prev_error1_2;
        dif_error2_2 = error2_2 - prev_error2_2;

        c1_2 = Kp1 * error1_2 + Kd1 * (dif_error1_2);
        c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

        prev_error1_2 = error1_2;
        prev_error2_2 = error2_2;

        correction1_2 = map(abs(c1_2), 0, 30, 0, 40);
        correction2_2 = map(abs(c2_2), 0, 40, 0, 60);

 
        if (error1_1 < 0 ){
          upr_mtr_fwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }
        
        else if (error1_1 > 0){
          upr_mtr_bwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }

        if (error2_1 < 0){
          lwr_mtr_fwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }
        
        else if (error2_1 > 0){
          lwr_mtr_bwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }

        if (error1_2 < 0 ){
          upr_mtr_fwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }
        
        else if (error1_2 > 0){
          upr_mtr_bwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }

        if (error2_2 < 0){
          lwr_mtr_fwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
        
        else if (error2_2 > 0){
          lwr_mtr_bwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
     
    }

        for (float u=0 ; u < 16.1 ; u = u + 0.8)
        {

      float xe_2 = -12+2.5*u;
      float ye_2 = -40;

      float xe_1 = -4 + 2.5*u ;
      float ye_1 = -40 ;

      if ( counter1_1 != temp1_1 ){
        temp1_1 = counter1_1;

        if (counter1_1 > 600)
        {
          counter1_1 = 0;
        }
        theta1c_1 = (counter1_1 * 0.6);
      }

      if ( counter2_1 != temp2_1 ){
        temp2_1 = counter2_1;

        if (counter2_1 > 600)
        {
          counter2_1 = 0;
        }
        theta2c_1 = (counter2_1 * 0.6);
      }

      if ( counter1_2 != temp1_2 ){
        temp1_2 = counter1_2;

        if (counter1_2 > 600){
          counter1_2 = 0;
          }

        theta1c_2 = (counter1_2 * 0.6);
      }

      if ( counter2_2 != temp2_2 ){
        temp2_2 = counter2_2;

        if (counter2_2 > 600)
        {
          counter2_2 = 0;
        }
        theta2c_2 = -(counter2_2 * 0.6);
      }


        if (atan(ye_1/xe_1)>0)
        {
          alpha_1 = atan(ye_1 / xe_1) - 3.14159;
        }
        else
        {
          alpha_1 = atan(ye_1 / xe_1);
        }

        if (atan(ye_2 / xe_2) > 0)
        {
          alpha_2 = atan(ye_2 / xe_2) - 3.14159;
        }
        else
         { alpha_2 = atan(ye_2 / xe_2);
         }
    
        theta1_1 = 57.2958 * (cosine_rule(l1,l2, sqrt(xe_1 * xe_1 + ye_1 * ye_1)) + alpha_1);
        theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

        error1_1 = theta1_1 - theta1c_1 + 39.94;            
        error2_1 = theta2_1 - theta2c_1/4 + 66.72;

        dif_error1_1 = error1_1 - prev_error1_1;
        dif_error2_1 = error2_1 - prev_error2_1;
       
        c1_1 = Kp1 * error1_1 + Kd1 * (dif_error1_1);
        c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

        prev_error1_1 = error1_1;
        prev_error2_1 = error2_1;

        correction1_1 = map(abs(c1_1), 0, 30, 0, 50);
        correction2_1 = map(abs(c2_1), 0, 40, 0, 50);

        Serial.print("x=");
        Serial.println(xe_1);       
        Serial.print("y=");
        Serial.println(ye_1);
        Serial.print("theta1_1=");
        Serial.println(theta1_1);
        Serial.print("theta1c_1=");
        Serial.println(theta1c_1-39.94);
        Serial.print("theta2c_1=");
        Serial.println(theta2c_1-66.72);        
        Serial.print("theta2_1=");
        Serial.println(theta2_1);
        Serial.print("c1_1=");
        Serial.println(c1_1);
        Serial.print("c2_1=");
        Serial.println(c2_1);
        Serial.print("pwm1=");
        Serial.println(correction1_1);
        Serial.print("pwm2=");
        Serial.println(correction2_1);
        Serial.println("------------------------");

        theta1_2 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_2 * xe_2) + (ye_2 * ye_2))) + alpha_2);
        theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

        error1_2 = theta1_2 - theta1c_2 + 47.8;         
        error2_2 = theta2_2 - theta2c_2/4 + 72.97;

        dif_error1_2 = error1_2 - prev_error1_2;
        dif_error2_2 = error2_2 - prev_error2_2;

        c1_2 = Kp1 * error1_2 + Kd1 * (dif_error1_2);
        c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

        prev_error1_2 = error1_2;
        prev_error2_2 = error2_2;

        correction1_2 = map(abs(c1_2), 0, 30, 0, 50);
        correction2_2 = map(abs(c2_2), 0, 40, 0, 50);

 
        if (error1_1 < 0 ){
          upr_mtr_fwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }
        
        else if (error1_1 > 0){
          upr_mtr_bwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }

        if (error2_1 < 0){
          lwr_mtr_fwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }
        
        else if (error2_1 > 0){
          lwr_mtr_bwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }

        if (error1_2 < 0 ){
          upr_mtr_fwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }
        
        else if (error1_2 > 0){
          upr_mtr_bwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }

        if (error2_2 < 0){
          lwr_mtr_fwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
        
        else if (error2_2 > 0){
          lwr_mtr_bwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
     
    }

}


  float cosine_rule(float c, float b, float a){
    float x = ( a * a + b * b - c * c ) / ( 2 * a * b );
    return acos(x);
  }

  void upr_mtr_fwd_1(){
    digitalWrite(motor1_1, HIGH);
  }

  void upr_mtr_bwd_1(){
    digitalWrite(motor1_1, LOW);
  }

  void lwr_mtr_fwd_1(){
    digitalWrite(motor2_1, HIGH);
  }

  void lwr_mtr_bwd_1(){
    digitalWrite(motor2_1, LOW);
  }

  void ai2_1() {
    if (digitalRead(6) == LOW)
    {
      counter1_1++;
    } else {
      counter1_1--;
    }
  }

  void ai3_1() {
    if (digitalRead(7) == LOW) {
      counter2_1++;
    } else {
      counter2_1--;
    }
  }

  void upr_mtr_fwd_2(){
    digitalWrite(motor1_2, HIGH);
  }

  void upr_mtr_bwd_2(){
    digitalWrite(motor1_2, LOW);
  }

  void lwr_mtr_fwd_2(){
    digitalWrite(motor2_2, HIGH);
  }

  void lwr_mtr_bwd_2(){
    digitalWrite(motor2_2, LOW);
  }

  void ai4_2() {
    if (digitalRead(9) == LOW)
    {
      counter1_2++;
    } else {
      counter1_2--;
    }
  }

  void ai5_2() {
    if (digitalRead(8) == LOW) {
      counter2_2++;
    } else {
      counter2_2--;
    }
  }
