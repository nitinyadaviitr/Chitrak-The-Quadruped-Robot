#define motor1_3  27                 
#define motor2_3  26
#define motor1pwm_3  3
#define motor2pwm_3  2

#define motor1_4  30
#define motor2_4  29
#define motor1pwm_4  5
#define motor2pwm_4  4

#include <SoftwareSerial.h>

SoftwareSerial mySerial1(10, 11); // RX, TX          

int bb=1;

float w = 0.1, alpha_3, theta1c_3 = 0.0 , theta2c_3 = 0.0, theta1_3, theta2_3, error1_3, error2_3, correction1_3, correction2_3, c1_3, c2_3;
float dif_error1_3 , prev_error1_3 = 0.0 , dif_error2_3 , prev_error2_3 = 0.0;

float alpha_4, theta1c_4 = 0.0 , theta2c_4 = 0.0, theta1_4, theta2_4, error1_4, error2_4, correction1_4, correction2_4, c1_4, c2_4;
float dif_error1_4 , prev_error1_4 = 0.0 , dif_error2_4 , prev_error2_4 = 0.0;

float Kp1 = 1.5, Kp2 = 1.5, Kd1 = 2.0, Kd2 = 2.0 ;

int l1 = 25, l2 = 25, a = 20, b = 7;

volatile int temp1_3 , counter1_3 = 0;
volatile int temp2_3 , counter2_3 = 0;

volatile int temp1_4 , counter1_4 = 0;
volatile int temp2_4 , counter2_4 = 0;


void setup()
{
  Serial.begin(9600);
  mySerial1.begin(9600);

//  LEG-3

  pinMode(21, INPUT_PULLUP);                                           
  pinMode(6, INPUT_PULLUP);
  attachInterrupt(2, ai2_3, RISING);
  
  pinMode(20, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(3, ai3_3, RISING);

  pinMode(motor1_3, OUTPUT);
  pinMode(motor1pwm_3, OUTPUT);
  pinMode(motor2_3, OUTPUT);
  pinMode(motor2pwm_3, OUTPUT);

  // LEG-4
  
  pinMode(19, INPUT_PULLUP);                                           
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(4, ai4_4, RISING);
  
  pinMode(18, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  attachInterrupt(5, ai5_4, RISING);

  pinMode(motor1_4, OUTPUT);
  pinMode(motor1pwm_4, OUTPUT);
  pinMode(motor2_4, OUTPUT);
  pinMode(motor2pwm_4, OUTPUT);

}

void loop()
{
    for (float u = 0; u < 16.1 ; u = u + 0.8)
    {

      float xe_3 = -4 + 1*u ;
      float ye_3 = -40;

      float xe_4 = -12 + 1*u ;
      float ye_4 = -40 ;

      if ( counter1_3 != temp1_3 )
      {
        temp1_3 = counter1_3;

        if (counter1_3 > 600)
          counter1_3 = 0;

        theta1c_3 = (counter1_3 * 0.6);
      }

      if ( counter2_3 != temp2_3 )
      {
        temp2_3 = counter2_3;

        if (counter2_3 > 600)
          counter2_3 = 0;

        theta2c_3 = -(counter2_3 * 0.6);
      }

      if ( counter1_4 != temp1_4 )
      {
        temp1_4 = counter1_4;

        if (counter1_4 > 600)
          counter1_4 = 0;

        theta1c_4 = (counter1_4 * 0.6);
      }

      if ( counter2_4 != temp2_4 )
      {
        temp2_4 = counter2_4;

        if (counter2_4 > 600)
          counter2_4 = 0;

        theta2c_4 = -(counter2_4 * 0.6);
      }


     
        if (atan(ye_3 / xe_3) > 0)
          alpha_3 = atan(ye_3 / xe_3) - PI;

        else
          alpha_3 = atan(ye_3 / xe_3);

        if (atan(ye_4 / xe_4) > 0)
          alpha_4 = atan(ye_4 / xe_4) - PI;

        else
          alpha_4 = atan(ye_4 / xe_4);
      


     
        theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
        theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

        error1_3 = theta1_3 - theta1c_3 + 59.22;        
        error2_3 = theta2_3 - theta2c_3/4 + 72.97;           

        dif_error1_3 = error1_3 - prev_error1_3;
        prev_error1_3 = error1_3;
        dif_error2_3 = error2_3 - prev_error2_3;
        prev_error2_3 = error2_3;

        c1_3 = Kp1 * error1_3 + Kd1 * (dif_error1_3);
        c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

        correction1_3 = map(abs(c1_3), 0, 30, 0, 50);
        correction2_3 = map(abs(c2_3), 0, 40, 0, 40);

        Serial.print("theta1_4=");
        Serial.println(theta1_3);
        Serial.print("theta1c_4=");
        Serial.println(theta1c_3);
        Serial.print("theta2_4=");
        Serial.println(theta2_3);
        Serial.print("theta2c_4=");
        Serial.println(theta2c_3);
        Serial.println("------------------------");
        Serial.print("c1_3=");
        Serial.println(c1_3);
        Serial.print("c2_3=");
        Serial.println(c2_3);

        theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
        theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

        error1_4 = theta1_4 - theta1c_4 + 73.34;         
        error2_4 = theta2_4 - theta2c_4/4 + 66.72;              

        dif_error1_4 = error1_4 - prev_error1_4;
        prev_error1_4 = error1_4;
        dif_error2_4 = error2_4 - prev_error2_4;
        prev_error2_4 = error2_4;

        c1_4 = Kp1 * error1_4 + Kd1 * (dif_error1_4);
        c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

        correction1_4 = map(abs(c1_4), 0, 30, 0, 50);
        correction2_4 = map(abs(c2_4), 0, 40, 0, 40);
      

   
        if (error1_3 < 0 )
        {
          upr_mtr_fwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }
        else if (error1_3 > 0)
        {
          upr_mtr_bwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }

        if (error2_3 < 0)
        {
          lwr_mtr_fwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }
        else if (error2_3 > 0)
        {
          lwr_mtr_bwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }

        if (error1_4 < 0 )
        {
          upr_mtr_fwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }
        else if (error1_4 > 0)
        {
          upr_mtr_bwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }

        if (error2_4 < 0)
        {
          lwr_mtr_fwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
        else if (error2_4 > 0)
        {
          lwr_mtr_bwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
    }
    
    for (float t = 0, u = 0; t < 3.14159, u < 8.2 ; t = t + 0.3, u = u + 0.8)
    {

      float xe_3 = 12 * cos(t);
      float ye_3 = -40 + 8 * sin(t);

      float xe_4 = 4 + 1*u ;
      float ye_4 = -40 ;

      if ( counter1_3 != temp1_3 )
      {
        temp1_3 = counter1_3;

        if (counter1_3 > 600)
          counter1_3 = 0;

        theta1c_3 = (counter1_3 * 0.6);
      }

      if ( counter2_3 != temp2_3 )
      {
        temp2_3 = counter2_3;

        if (counter2_3 > 600)
          counter2_3 = 0;

        theta2c_3 = -(counter2_3 * 0.6);
      }

      if ( counter1_4 != temp1_4 )
      {
        temp1_4 = counter1_4;

        if (counter1_4 > 600)
          counter1_4 = 0;

        theta1c_4 = (counter1_4 * 0.6);
      }

      if ( counter2_4 != temp2_4 )
      {
        temp2_4 = counter2_4;

        if (counter2_4 > 600)
          counter2_4 = 0;

        theta2c_4 = -(counter2_4 * 0.6);
      }


     
        if (atan(ye_3 / xe_3) > 0)
          alpha_3 = atan(ye_3 / xe_3) - PI;

        else
          alpha_3 = atan(ye_3 / xe_3);

        if (atan(ye_4 / xe_4) > 0)
          alpha_4 = atan(ye_4 / xe_4) - PI;

        else
          alpha_4 = atan(ye_4 / xe_4);
      


     
        theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
        theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

        error1_3 = theta1_3 - theta1c_3 + 59.22;       
        error2_3 = theta2_3 - theta2c_3/4 + 72.97;        

        dif_error1_3 = error1_3 - prev_error1_3;
        prev_error1_3 = error1_3;
        dif_error2_3 = error2_3 - prev_error2_3;
        prev_error2_3 = error2_3;

        c1_3 = Kp1 * error1_3 + Kd1 * (dif_error1_3);
        c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

        correction1_3 = map(abs(c1_3), 0, 30, 0, 40);
        correction2_3 = map(abs(c2_3), 0, 40, 0, 60);

        Serial.print("theta1_4=");
        Serial.println(theta1_3);
        Serial.print("theta1c_4=");
        Serial.println(theta1c_3);
        Serial.print("theta2_4=");
        Serial.println(theta2_3);
        Serial.print("theta2c_4=");
        Serial.println(theta2c_3);
        Serial.println("------------------------");
        Serial.print("c1_3=");
        Serial.println(c1_3);
        Serial.print("c2_3=");
        Serial.println(c2_3);

        theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
        theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

        error1_4 = theta1_4 - theta1c_4 + 73.34;        
        error2_4 = theta2_4 - theta2c_4/4 + 66.72;            

        dif_error1_4 = error1_4 - prev_error1_4;
        prev_error1_4 = error1_4;
        dif_error2_4 = error2_4 - prev_error2_4;
        prev_error2_4 = error2_4;

        c1_4 = Kp1 * error1_4 + Kd1 * (dif_error1_4);
        c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

        correction1_4 = map(abs(c1_4), 0, 30, 0, 50);
        correction2_4 = map(abs(c2_4), 0, 40, 0, 40);
      

   
        if (error1_3 < 0 ){
          upr_mtr_fwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }
        
        else if (error1_3 > 0)
        {
          upr_mtr_bwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }

        if (error2_3 < 0)
        {
          lwr_mtr_fwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }
        else if (error2_3 > 0)
        {
          lwr_mtr_bwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }

        if (error1_4 < 0 )
        {
          upr_mtr_fwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }
        else if (error1_4 > 0)
        {
          upr_mtr_bwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }

        if (error2_4 < 0)
        {
          lwr_mtr_fwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
        else if (error2_4 > 0)
        {
          lwr_mtr_bwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
    }

    for (float t = 0, u = 0; t < 3.14159, u < 8.2 ; t = t + 0.3, u = u + 0.8)
    {

      float xe_4 = 12 * cos(t);
      float ye_4 = -40 + 8 * sin(t);

      float xe_3 = -12 + 1 * u ;
      float ye_3 = -40 ;

      if ( counter1_3 != temp1_3 )
      {
        temp1_3 = counter1_3;

        if (counter1_3 > 600)
          counter1_3 = 0;

        theta1c_3 = (counter1_3 * 0.6);
      }

      if ( counter2_3 != temp2_3 )
      {
        temp2_3 = counter2_3;

        if (counter2_3 > 600)
          counter2_3 = 0;

        theta2c_3 = -(counter2_3 * 0.6);
      }

      if ( counter1_4 != temp1_4 )
      {
        temp1_4 = counter1_4;

        if (counter1_4 > 600)
          counter1_4 = 0;

        theta1c_4 = (counter1_4 * 0.6);
      }

      if ( counter2_4 != temp2_4 )
      {
        temp2_4 = counter2_4;

        if (counter2_4 > 600)
          counter2_4 = 0;

        theta2c_4 = -(counter2_4 * 0.6);
      }


     
        if (atan(ye_3 / xe_3) > 0)
          alpha_3 = atan(ye_3 / xe_3) - PI;

        else
          alpha_3 = atan(ye_3 / xe_3);

        if (atan(ye_4 / xe_4) > 0)
          alpha_4 = atan(ye_4 / xe_4) - PI;

        else
          alpha_4 = atan(ye_4 / xe_4);
      


     
        theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
        theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

        error1_3 = theta1_3 - theta1c_3 + 59.22;        
        error2_3 = theta2_3 - theta2c_3/4 + 72.97;            

        dif_error1_3 = error1_3 - prev_error1_3;
        prev_error1_3 = error1_3;
        dif_error2_3 = error2_3 - prev_error2_3;
        prev_error2_3 = error2_3;

        c1_3 = Kp1 * error1_3 + Kd1 * (dif_error1_3);
        c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

        correction1_3 = map(abs(c1_3), 0, 30, 0, 50);
        correction2_3 = map(abs(c2_3), 0, 40, 0, 40);

        Serial.print("theta1_4=");
        Serial.println(theta1_3);
        Serial.print("theta1c_4=");
        Serial.println(theta1c_3);
        Serial.print("theta2_4=");
        Serial.println(theta2_3);
        Serial.print("theta2c_4=");
        Serial.println(theta2c_3);
        Serial.println("------------------------");
        Serial.print("c1_3=");
        Serial.println(c1_3);
        Serial.print("c2_3=");
        Serial.println(c2_3);

        theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
        theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

        error1_4 = theta1_4 - theta1c_4 + 73.34;         
        error2_4 = theta2_4 - theta2c_4/4 + 66.72;              

        dif_error1_4 = error1_4 - prev_error1_4;
        prev_error1_4 = error1_4;
        dif_error2_4 = error2_4 - prev_error2_4;
        prev_error2_4 = error2_4;

        c1_4 = Kp1 * error1_4 + Kd1 * (dif_error1_4);
        c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

        correction1_4 = map(abs(c1_4), 0, 30, 0, 40);
        correction2_4 = map(abs(c2_4), 0, 40, 0, 60);
      

   
        if (error1_3 < 0 )
        {
          upr_mtr_fwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }
        else if (error1_3 > 0)
        {
          upr_mtr_bwd_3();
          analogWrite(motor1pwm_3, abs(correction1_3));
        }

        if (error2_3 < 0)
        {
          lwr_mtr_fwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }
        else if (error2_3 > 0)
        {
          lwr_mtr_bwd_3();
          analogWrite(motor2pwm_3, abs(correction2_3));
        }

        if (error1_4 < 0 )
        {
          upr_mtr_fwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }
        else if (error1_4 > 0)
        {
          upr_mtr_bwd_4();
          analogWrite(motor1pwm_4, abs(correction1_4));
        }

        if (error2_4 < 0)
        {
          lwr_mtr_fwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
        else if (error2_4 > 0)
        {
          lwr_mtr_bwd_4();
          analogWrite(motor2pwm_4, abs(correction2_4));
        }
    }

}

  float cosine_rule(float c, float b, float a){
    float x = ( a * a + b * b - c * c ) / ( 2 * a * b );
    return acos(x);
  }

  void upr_mtr_fwd_3(){
    digitalWrite(motor1_3, HIGH);
  }

  void upr_mtr_bwd_3(){
    digitalWrite(motor1_3, LOW);
  }

  void lwr_mtr_fwd_3(){
    digitalWrite(motor2_3, HIGH);
  }

  void lwr_mtr_bwd_3(){
    digitalWrite(motor2_3, LOW);
  }

  void ai2_3(){
    if (digitalRead(6) == LOW)
    {
      counter1_3++;
    } else {
      counter1_3--;
    }
  }

  void ai3_3() {
  if (digitalRead(7) == LOW) {
      counter2_3++;
    } else {
      counter2_3--;
    }
  }

  void upr_mtr_fwd_4(){
    digitalWrite(motor1_4, HIGH);
  }

  void upr_mtr_bwd_4(){
    digitalWrite(motor1_4, LOW);
  }

  void lwr_mtr_fwd_4(){
    digitalWrite(motor2_4, HIGH);
  }

  void lwr_mtr_bwd_4(){
    digitalWrite(motor2_4, LOW);
  }

  void ai4_4(){
    if (digitalRead(8) == LOW)
    {
      counter1_4++;
    } else {
      counter1_4--;
    }
  }

  void ai5_4() {
  if (digitalRead(9) == LOW) {
      counter2_4++;
    } else {
      counter2_4--;
    }
  }

