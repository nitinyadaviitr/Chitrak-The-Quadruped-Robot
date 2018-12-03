#define motor1_1  6
#define motor2_1  4
#define motor1pwm_1  10
#define motor2pwm_1  11

#define motor1_2  6
#define motor2_2  4
#define motor1pwm_2  10
#define motor2pwm_2  11

#include <SoftwareSerial.h>

SoftwareSerial mySerial4(10, 11); // RX, TX          NEED TO CHANGE PINS


{   // VARIABLES
float w = 0.1, alpha_1, theta1c_1 = 0.0 , theta2c_1 = 0.0, theta1_1, theta2_1, error1_1, error2_1, correction1_1, correction2_1, c1_1, c2_1;
float dif_error1_1 , prev_error1_1 = 0.0 , dif_error2_1 , prev_error2_1 = 0.0;

float w = 0.1, alpha_2, theta1c_2 = 0.0 , theta2c_2 = 0.0, theta1_2, theta2_2, error1_2, error2_2, correction1_2, correction2_2, c1_2, c2_2;
float dif_error1_2 , prev_error1_2 = 0.0 , dif_error2_2 , prev_error2_2 = 0.0;

float Kp1 = 4.5, Kp2 = 4.5, Kd1 = 0.8, Kd2 = 0.8 ;

int l1 = 26, l2 = 22, a = 20, b = 7;

volatile int temp1_1 , counter1_1 = 0;
volatile int temp2_1 , counter2_1 = 0;

volatile int temp1_2 , counter1_2 = 0;
volatile int temp2_2 , counter2_2 = 0;
}

void setup()
{
  Serial.begin(9600);
  mySerial4.begin(115200);

  pinMode(2, INPUT_PULLUP);                                           // NEED TO WRITE FOR LEG 2
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, ai0_1, RISING);
  attachInterrupt(1, ai1_1, RISING);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(2, ai2_1, RISING);
  attachInterrupt(3, ai3_1, RISING);

  pinMode(motor1_1, OUTPUT);
  pinMode(motor1pwm_1, OUTPUT);
  pinMode(motor2_1, OUTPUT);
  pinMode(motor2pwm_1, OUTPUT);

}

void loop()
{
  if ( mySerial4.available() && mySerial4.read() == 4)
  {
    for (float t = 0, u = 0; t < 3.14159, u < 16 ; t = t + 0.08, u = u + 3)
    {

      float xe_1 = 24 * cos(t);
      float ye_1 = -40 + 13 * sin(t);

      float xe_2 = 8 + u ;
      float ye_2 = -40 ;

      if ( counter1_1 != temp1_1 )
      {
        temp1_1 = counter1_1;

        if (counter1_1 > 1200)
          counter1_1 = 0;

        theta1c_1 = - (counter1_1 * 0.3);
      }

      if ( counter2_1 != temp2_1 )
      {
        temp2_1 = counter2_1;

        if (counter2_1 > 1200)
          counter2_1 = 0;

        theta2c_1 = (counter2_1 * 0.3);
      }

      if ( counter1_2 != temp1_2 )
      {
        temp1_2 = counter1_2;

        if (counter1_2 > 1200)
          counter1_2 = 0;

        theta1c_2 = - (counter1_2 * 0.3);
      }

      if ( counter2_2 != temp2_2 )
      {
        temp2_2 = counter2_2;

        if (counter2_2 > 1200)
          counter2_2 = 0;

        theta2c_2 = (counter2_2 * 0.3);
      }


      { //   CALCULATE ALPHA
        if (atan(ye_1 / xe_1) > 0)
          alpha_1 = atan(ye_1 / xe_1) - PI;

        else
          alpha_1 = atan(ye_1 / xe_1);

        if (atan(ye_2 / xe_2) > 0)
          alpha_2 = atan(ye_2 / xe_2) - PI;

        else
          alpha_2 = atan(ye_2 / xe_2);
      }


      { //   CALCULATE CORRECTION
        theta1_1 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_1 * xe_1) + (ye_1 * ye_1))) + alpha_1);
        theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

        error1_1 = theta1_1 - theta1c_1 + 45;
        error2_1 = theta2_1 - theta2c_1;

        dif_error1_1 = error1_1 - prev_error1_1;
        prev_error1_1 = error1_1;
        dif_error2_1 = error2_1 - prev_error2_1;
        prev_error2_1 = error2_1;

        c1_1 = Kp1 * error1_1 + Kd1 * (dif_error1_1);
        c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

        correction1_1 = map(abs(c1_1), 0, 150, 0, 100);
        correction2_1 = map(abs(c2_1), 0, 250, 0, 100);

        theta1_2 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_2 * xe_2) + (ye_2 * ye_2))) + alpha_2);
        theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

        error1_2 = theta1_2 - theta1c_2 + 45;
        error2_2 = theta2_2 - theta2c_2;

        dif_error1_2 = error1_2 - prev_error1_2;
        prev_error1_2 = error1_2;
        dif_error2_2 = error2_2 - prev_error2_2;
        prev_error2_2 = error2_2;

        c1_2 = Kp1 * error1_2 + Kd1 * (dif_error1_2);
        c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

        correction1_2 = map(abs(c1_2), 0, 150, 0, 100);
        correction2_2 = map(abs(c2_2), 0, 250, 0, 100);
      }

      { //   ACTUATE
        if (error1_1 < 0 )
        {
          upr_mtr_fwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }
        else if (error1_1 > 0)
        {
          upr_mtr_bwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }

        if (error2_1 < 0)
        {
          lwr_mtr_fwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }
        else if (error2_1 > 0)
        {
          lwr_mtr_bwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }

        if (error1_2 < 0 )
        {
          upr_mtr_fwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }
        else if (error1_2 > 0)
        {
          upr_mtr_bwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }

        if (error2_2 < 0)
        {
          lwr_mtr_fwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
        else if (error2_2 > 0)
        {
          lwr_mtr_bwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
      }


    }

    //      char str[4] = "end4";

    //      mySerial4.write(str, 4);
  }

  else if()
  {
    for (float t = 0, u = 0; t < 3.14159, u < 16 ; t = t + 0.08, u = u + 3)
    {

      float xe_2 = 24 * cos(t);
      float ye_2 = -40 + 13 * sin(t);

      float xe_1 = -24 + u ;
      float ye_1 = -40 ;

      if ( counter1_1 != temp1_1 )
      {
        temp1_1 = counter1_1;

        if (counter1_1 > 1200)
          counter1_1 = 0;

        theta1c_1 = - (counter1_1 * 0.3);
      }

      if ( counter2_1 != temp2_1 )
      {
        temp2_1 = counter2_1;

        if (counter2_1 > 1200)
          counter2_1 = 0;

        theta2c_1 = (counter2_1 * 0.3);
      }

      if ( counter1_2 != temp1_2 )
      {
        temp1_2 = counter1_2;

        if (counter1_2 > 1200)
          counter1_2 = 0;

        theta1c_2 = - (counter1_2 * 0.3);
      }

      if ( counter2_2 != temp2_2 )
      {
        temp2_2 = counter2_2;

        if (counter2_2 > 1200)
          counter2_2 = 0;

        theta2c_2 = (counter2_2 * 0.3);
      }


      { //   CALCULATE ALPHA
        if (atan(ye_1 / xe_1) > 0)
          alpha_1 = atan(ye_1 / xe_1) - PI;

        else
          alpha_1 = atan(ye_1 / xe_1);

        if (atan(ye_2 / xe_2) > 0)
          alpha_2 = atan(ye_2 / xe_2) - PI;

        else
          alpha_2 = atan(ye_2 / xe_2);
      }


      { //   CALCULATE CORRECTION
        theta1_1 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_1 * xe_1) + (ye_1 * ye_1))) + alpha_1);
        theta2_1 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_1 * xe_1 + ye_1 * ye_1), l1, l2));

        error1_1 = theta1_1 - theta1c_1 + 45;
        error2_1 = theta2_1 - theta2c_1;

        dif_error1_1 = error1_1 - prev_error1_1;
        prev_error1_1 = error1_1;
        dif_error2_1 = error2_1 - prev_error2_1;
        prev_error2_1 = error2_1;

        c1_1 = Kp1 * error1_1 + Kd1 * (dif_error1_1);
        c2_1 = Kp2 * error2_1 + Kd2 * (dif_error2_1);

        correction1_1 = map(abs(c1_1), 0, 150, 0, 100);
        correction2_1 = map(abs(c2_1), 0, 250, 0, 100);

        theta1_2 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_2 * xe_2) + (ye_2 * ye_2))) + alpha_2);
        theta2_2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_2 * xe_2 + ye_2 * ye_2), l1, l2));

        error1_2 = theta1_2 - theta1c_2 + 45;
        error2_2 = theta2_2 - theta2c_2;

        dif_error1_2 = error1_2 - prev_error1_2;
        prev_error1_2 = error1_2;
        dif_error2_2 = error2_2 - prev_error2_2;
        prev_error2_2 = error2_2;

        c1_2 = Kp1 * error1_2 + Kd1 * (dif_error1_2);
        c2_2 = Kp2 * error2_2 + Kd2 * (dif_error2_2);

        correction1_2 = map(abs(c1_2), 0, 150, 0, 100);
        correction2_2 = map(abs(c2_2), 0, 250, 0, 100);
      }

      { //   ACTUATE
        if (error1_1 < 0 )
        {
          upr_mtr_fwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }
        else if (error1_1 > 0)
        {
          upr_mtr_bwd_1();
          analogWrite(motor1pwm_1, abs(correction1_1));
        }

        if (error2_1 < 0)
        {
          lwr_mtr_fwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }
        else if (error2_1 > 0)
        {
          lwr_mtr_bwd_1();
          analogWrite(motor2pwm_1, abs(correction2_1));
        }

        if (error1_2 < 0 )
        {
          upr_mtr_fwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }
        else if (error1_2 > 0)
        {
          upr_mtr_bwd_2();
          analogWrite(motor1pwm_2, abs(correction1_2));
        }

        if (error2_2 < 0)
        {
          lwr_mtr_fwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
        else if (error2_2 > 0)
        {
          lwr_mtr_bwd_2();
          analogWrite(motor2pwm_2, abs(correction2_2));
        }
      }


    }

    //      char str[4] = "end4";

    //      mySerial4.write(str, 4);
  }
}

{ //FUNCTIONS
  float cosine_rule(float c, float b, float a)
  {
    float x = ( a * a + b * b - c * c ) / ( 2 * a * b );
    return acos(x);
  }

  void upr_mtr_fwd_1()
  {
    digitalWrite(motor1_1, HIGH);
  }

  void upr_mtr_bwd_1()
  {
    digitalWrite(motor1_1, LOW);
  }

  void lwr_mtr_fwd_1()
  {
    digitalWrite(motor2_1, HIGH);
  }

  void lwr_mtr_bwd_1()
  {
    digitalWrite(motor2_1, LOW);
  }

  void ai0_1()
  {
    if (digitalRead(3) == LOW)
    {
      counter1_1++;
    } else {
      counter1_1--;
    }
  }

  void ai1_1() {
    if (digitalRead(2) == LOW)
    {
      counter1_1--;
    } else {
      counter1_1++;
    }
  }

  void ai2()_1 {
    if (digitalRead(20) == LOW) {
      counter2_1++;
    } else {
      counter2_1--;
    }
  }

  void ai3()_1 {
    if (digitalRead(21) == LOW) {
      counter2_1--;
    } else {
      counter2_1++;
    }
  }

  void upr_mtr_fwd_2()
  {
    digitalWrite(motor1_2, HIGH);
  }

  void upr_mtr_bwd_2()
  {
    digitalWrite(motor1_2, LOW);
  }

  void lwr_mtr_fwd_2()
  {
    digitalWrite(motor2_2, HIGH);
  }

  void lwr_mtr_bwd_2()
  {
    digitalWrite(motor2_2, LOW);
  }

  void ai0_2()
  {
    if (digitalRead(3) == LOW)
    {
      counter1_2++;
    } else {
      counter1_2--;
    }
  }

  void ai1_2() {
    if (digitalRead(2) == LOW)
    {
      counter1_2--;
    } else {
      counter1_2++;
    }
  }

  void ai2()_2 {
    if (digitalRead(20) == LOW) {
      counter2_2++;
    } else {
      counter2_2--;
    }
  }

  void ai3()_2 {
    if (digitalRead(21) == LOW) {
      counter2_2--;
    } else {
      counter2_2++;
    }
  }
}
