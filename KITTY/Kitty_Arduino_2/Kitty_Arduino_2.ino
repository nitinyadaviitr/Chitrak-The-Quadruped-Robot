#define motor1_3  6                     //                                1-hip    2-knee
#define motor2_3  4
#define motor1pwm_3  7
#define motor2pwm_3  5

#define motor1_4  16
#define motor2_4  14
#define motor1pwm_4  17
#define motor2pwm_4  15

#include <SoftwareSerial.h>

SoftwareSerial mySerial4(10, 11); // RX, TX          NEED TO CHANGE PINS


{   // VARIABLES
float w = 0.1, alpha_3, theta1c_3 = 0.0 , theta2c_3 = 0.0, theta1_3, theta2_3, error1_3, error2_3, correction1_3, correction2_3, c1_3, c2_3;
float dif_error1_3 , prev_error1_3 = 0.0 , dif_error2_3 , prev_error2_3 = 0.0;

float w = 0.1, alpha_4, theta1c_4 = 0.0 , theta2c_4 = 0.0, theta1_4, theta2_4, error1_4, error2_4, correction1_4, correction2_4, c1_4, c2_4;
float dif_error1_4 , prev_error1_4 = 0.0 , dif_error2_4 , prev_error2_4 = 0.0;

float Kp1 = 4.5, Kp2 = 4.5, Kd1 = 0.8, Kd2 = 0.8 ;

int l1 = 26, l2 = 22, a = 20, b = 7;

volatile int temp1_3 , counter1_3 = 0;
volatile int temp2_3 , counter2_3 = 0;

volatile int temp1_4 , counter1_4 = 0;
volatile int temp2_4 , counter2_4 = 0;
}

void setup()
{
  Serial.begin(9600);
  mySerial4.begin(115200);

  pinMode(2, INPUT_PULLUP);                                           // NEED TO WRITE FOR LEG 2
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, ai0_3, RISING);
  attachInterrupt(1, ai1_3, RISING);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(2, ai2_3, RISING);
  attachInterrupt(3, ai3_3, RISING);

  pinMode(motor1_3, OUTPUT);
  pinMode(motor1pwm_3, OUTPUT);
  pinMode(motor2_3, OUTPUT);
  pinMode(motor2pwm_3, OUTPUT);

}

void loop()
{
  if ( mySerial4.available() && mySerial4.read() == 4)
  {
    for (float t = 0, u = 0; t < 3.14159, u < 16 ; t = t + 0.08, u = u + 3)
    {

      float xe_3 = 24 * cos(t);
      float ye_3 = -40 + 13 * sin(t);

      float xe_4 = 8 + u ;
      float ye_4 = -40 ;

      if ( counter1_3 != temp1_3 )
      {
        temp1_3 = counter1_3;

        if (counter1_3 > 1200)
          counter1_3 = 0;

        theta1c_3 = - (counter1_3 * 0.3);
      }

      if ( counter2_3 != temp2_3 )
      {
        temp2_3 = counter2_3;

        if (counter2_3 > 1200)
          counter2_3 = 0;

        theta2c_3 = (counter2_3 * 0.3);
      }

      if ( counter1_4 != temp1_4 )
      {
        temp1_4 = counter1_4;

        if (counter1_4 > 1200)
          counter1_4 = 0;

        theta1c_4 = - (counter1_4 * 0.3);
      }

      if ( counter2_4 != temp2_4 )
      {
        temp2_4 = counter2_4;

        if (counter2_4 > 1200)
          counter2_4 = 0;

        theta2c_4 = (counter2_4 * 0.3);
      }


      { //   CALCULATE ALPHA
        if (atan(ye_3 / xe_3) > 0)
          alpha_3 = atan(ye_3 / xe_3) - PI;

        else
          alpha_3 = atan(ye_3 / xe_3);

        if (atan(ye_4 / xe_4) > 0)
          alpha_4 = atan(ye_4 / xe_4) - PI;

        else
          alpha_4 = atan(ye_4 / xe_4);
      }


      { //   CALCULATE CORRECTION
        theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
        theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

        error1_3 = theta1_3 - theta1c_3 + 45;
        error2_3 = theta2_3 - theta2c_3;

        dif_error1_3 = error1_3 - prev_error1_3;
        prev_error1_3 = error1_3;
        dif_error2_3 = error2_3 - prev_error2_3;
        prev_error2_3 = error2_3;

        c1_3 = Kp1 * error1_3 + Kd1 * (dif_error1_3);
        c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

        correction1_3 = map(abs(c1_3), 0, 150, 0, 100);
        correction2_3 = map(abs(c2_3), 0, 250, 0, 100);

        theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
        theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

        error1_4 = theta1_4 - theta1c_4 + 45;
        error2_4 = theta2_4 - theta2c_4;

        dif_error1_4 = error1_4 - prev_error1_4;
        prev_error1_4 = error1_4;
        dif_error2_4 = error2_4 - prev_error2_4;
        prev_error2_4 = error2_4;

        c1_4 = Kp1 * error1_4 + Kd1 * (dif_error1_4);
        c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

        correction1_4 = map(abs(c1_4), 0, 150, 0, 100);
        correction2_4 = map(abs(c2_4), 0, 250, 0, 100);
      }

      { //   ACTUATE
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

    //      char str[4] = "end4";

    //      mySerial4.write(str, 4);
  }

  else if()
  {
    for (float t = 0, u = 0; t < 3.14159, u < 16 ; t = t + 0.08, u = u + 3)
    {

      float xe_4 = 24 * cos(t);
      float ye_4 = -40 + 13 * sin(t);

      float xe_3 = -24 + u ;
      float ye_3 = -40 ;

      if ( counter1_3 != temp1_3 )
      {
        temp1_3 = counter1_3;

        if (counter1_3 > 1200)
          counter1_3 = 0;

        theta1c_3 = - (counter1_3 * 0.3);
      }

      if ( counter2_3 != temp2_3 )
      {
        temp2_3 = counter2_3;

        if (counter2_3 > 1200)
          counter2_3 = 0;

        theta2c_3 = (counter2_3 * 0.3);
      }

      if ( counter1_4 != temp1_4 )
      {
        temp1_4 = counter1_4;

        if (counter1_4 > 1200)
          counter1_4 = 0;

        theta1c_4 = - (counter1_4 * 0.3);
      }

      if ( counter2_4 != temp2_4 )
      {
        temp2_4 = counter2_4;

        if (counter2_4 > 1200)
          counter2_4 = 0;

        theta2c_4 = (counter2_4 * 0.3);
      }


      { //   CALCULATE ALPHA
        if (atan(ye_3 / xe_3) > 0)
          alpha_3 = atan(ye_3 / xe_3) - PI;

        else
          alpha_3 = atan(ye_3 / xe_3);

        if (atan(ye_4 / xe_4) > 0)
          alpha_4 = atan(ye_4 / xe_4) - PI;

        else
          alpha_4 = atan(ye_4 / xe_4);
      }


      { //   CALCULATE CORRECTION
        theta1_3 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_3 * xe_3) + (ye_3 * ye_3))) + alpha_3);
        theta2_3 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_3 * xe_3 + ye_3 * ye_3), l1, l2));

        error1_3 = theta1_3 - theta1c_3 + 45;
        error2_3 = theta2_3 - theta2c_3;

        dif_error1_3 = error1_3 - prev_error1_3;
        prev_error1_3 = error1_3;
        dif_error2_3 = error2_3 - prev_error2_3;
        prev_error2_3 = error2_3;

        c1_3 = Kp1 * error1_3 + Kd1 * (dif_error1_3);
        c2_3 = Kp2 * error2_3 + Kd2 * (dif_error2_3);

        correction1_3 = map(abs(c1_3), 0, 150, 0, 100);
        correction2_3 = map(abs(c2_3), 0, 250, 0, 100);

        theta1_4 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe_4 * xe_4) + (ye_4 * ye_4))) + alpha_4);
        theta2_4 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe_4 * xe_4 + ye_4 * ye_4), l1, l2));

        error1_4 = theta1_4 - theta1c_4 + 45;
        error2_4 = theta2_4 - theta2c_4;

        dif_error1_4 = error1_4 - prev_error1_4;
        prev_error1_4 = error1_4;
        dif_error2_4 = error2_4 - prev_error2_4;
        prev_error2_4 = error2_4;

        c1_4 = Kp1 * error1_4 + Kd1 * (dif_error1_4);
        c2_4 = Kp2 * error2_4 + Kd2 * (dif_error2_4);

        correction1_4 = map(abs(c1_4), 0, 150, 0, 100);
        correction2_4 = map(abs(c2_4), 0, 250, 0, 100);
      }

      { //   ACTUATE
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

  void upr_mtr_fwd_3()
  {
    digitalWrite(motor1_3, HIGH);
  }

  void upr_mtr_bwd_3()
  {
    digitalWrite(motor1_3, LOW);
  }

  void lwr_mtr_fwd_3()
  {
    digitalWrite(motor2_3, HIGH);
  }

  void lwr_mtr_bwd_3()
  {
    digitalWrite(motor2_3, LOW);
  }

  void ai0_3()
  {
    if (digitalRead(3) == LOW)
    {
      counter1_3++;
    } else {
      counter1_3--;
    }
  }

  void ai1_3() {
    if (digitalRead(2) == LOW)
    {
      counter1_3--;
    } else {
      counter1_3++;
    }
  }

  void ai2()_3 {
    if (digitalRead(20) == LOW) {
      counter2_3++;
    } else {
      counter2_3--;
    }
  }

  void ai3()_3 {
    if (digitalRead(21) == LOW) {
      counter2_3--;
    } else {
      counter2_3++;
    }
  }

  void upr_mtr_fwd_4()
  {
    digitalWrite(motor1_4, HIGH);
  }

  void upr_mtr_bwd_4()
  {
    digitalWrite(motor1_4, LOW);
  }

  void lwr_mtr_fwd_4()
  {
    digitalWrite(motor2_4, HIGH);
  }

  void lwr_mtr_bwd_4()
  {
    digitalWrite(motor2_4, LOW);
  }

  void ai0_4()
  {
    if (digitalRead(3) == LOW)
    {
      counter1_4++;
    } else {
      counter1_4--;
    }
  }

  void ai1_4() {
    if (digitalRead(2) == LOW)
    {
      counter1_4--;
    } else {
      counter1_4++;
    }
  }

  void ai2()_4 {
    if (digitalRead(20) == LOW) {
      counter2_4++;
    } else {
      counter2_4--;
    }
  }

  void ai3()_4 {
    if (digitalRead(21) == LOW) {
      counter2_4--;
    } else {
      counter2_4++;
    }
  }
}
