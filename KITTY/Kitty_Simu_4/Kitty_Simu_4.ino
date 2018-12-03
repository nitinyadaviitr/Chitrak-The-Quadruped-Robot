#define motor1  6
#define motor2  4
#define motor1pwm  10
#define motor2pwm  11

#include <SoftwareSerial.h>

SoftwareSerial mySerial4(10, 11); // RX, TX          NEED TO CHANGE PINS

float w = 0.1, alpha, theta1c = 0.0 , theta2c = 0.0, theta1, theta2, error1, error2, correction1, correction2, c1, c2;
float dif_error1 , prev_error1 = 0.0 , dif_error2 , prev_error2 = 0.0 , I = 0.0, l = 48.0;
float Kp1 = 4.5, Kp2 = 4.5, Kd1 = 0.8, Kd2 = 0.8 , Ki1 = 0.0;

int l1 = 26, l2 = 22, a = 20, b = 7;

volatile int temp1 , counter1 = 0;
volatile int temp2 , counter2 = 0;

int flag = 0;

//------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(9600);
  mySerial4.begin(115200);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(2, ai2, RISING);
  attachInterrupt(3, ai3, RISING);

  pinMode(motor1, OUTPUT);
  pinMode(motor1pwm, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor2pwm, OUTPUT);

}

void loop()
{

  if (flag == 1)
  {
    analogWrite(motor1pwm, 0);
    analogWrite(motor2pwm, 0);
    Serial.println("stopped");
  }

  if (flag == 0)
  {
    if ( mySerial4.available() && mySerial4.read() == 4)
    {
      for (float t = 0; t < 3.14159 ; t = t + 0.08)
      {

        float xe = 24 * cos(t);
        float ye = -40 + 13 * sin(t);

        if ( counter1 != temp1 )
        {
              temp1 = counter1;
              
              if (counter1 > 1200)
              counter1 = 0;
              
              theta1c = - (counter1 * 0.3);
        }
        
        if ( counter2 != temp2 )
        {
              temp2 = counter2;
              
              if (counter2 > 1200)
              counter2 = 0;
              
              theta2c = (counter2 * 0.3);
        }

        if (atan(ye / xe) > 0)
        alpha = atan(ye / xe) - PI;
        
        else
        alpha = atan(ye / xe);

        theta1 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe * xe) + (ye * ye))) + alpha);
        theta2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe * xe + ye * ye), l1, l2));

        error1 = theta1 - theta1c + 45;
        error2 = theta2 - theta2c;

        dif_error1 = error1 - prev_error1;
        prev_error1 = error1;
        dif_error2 = error2 - prev_error2;
        prev_error2 = error2;

        I += error1;
        c1 = Kp1 * error1 + Kd1 * (dif_error1);
        c2 = Kp2 * error2 + Kd2 * (dif_error2);

        correction1 = map(abs(c1), 0, 150, 0, 100);
        correction2 = map(abs(c2), 0, 250, 0, 100);

          if (error1 < 0 )
          {
            upr_mtr_fwd();
            analogWrite(motor1pwm, abs(correction1));
          }
          else if (error1 > 0)
          {
            upr_mtr_bwd();
            analogWrite(motor1pwm, abs(correction1));
          }
  
          if (error2 < 0)
          {
            lwr_mtr_fwd();
            analogWrite(motor2pwm, abs(correction2));
          }
          else if (error2 > 0)
          {
            lwr_mtr_bwd();
            analogWrite(motor2pwm, abs(correction2));
          }
      }

      char str[4] = "end4";

      mySerial4.write(str, 4);
    }

    else
    {
      for (float t = 0; t < 48 ; t = t + 3)
      {
        float xe = -24 + t;
        float ye = -40;

        if ( counter1 != temp1 )
        {
          temp1 = counter1;

          if (counter1 > 1200)
          {
            counter1 = 0;
          }
          theta1c = - (counter1 * 0.3);
        }

        if ( counter2 != temp2 )
        {
          temp2 = counter2;
          if (counter2 > 1200)
          {
            counter2 = 0;
          }
          theta2c = (counter2 * 0.3);
        }

        if (atan(ye / xe) > 0)
        {
          alpha = atan(ye / xe) - 3.14159;
        }
        else
        {
          alpha = atan(ye / xe);
        }

        theta1 = 57.2958 * (cosine_rule(l2, l1, sqrt((xe * xe) + (ye * ye))) + alpha);
        theta2 = 57.2958 * (-3.14159 + cosine_rule(sqrt(xe * xe + ye * ye), l1, l2));

        error1 = theta1 - theta1c + 45;
        error2 = theta2 - theta2c;

        dif_error1 = error1 - prev_error1;
        prev_error1 = error1;
        dif_error2 = error2 - prev_error2;
        prev_error2 = error2;

        c1 = Kp1 * error1 + Kd1 * (dif_error1);
        c2 = Kp2 * error2 + Kd2 * (dif_error2);

        correction1 = map(abs(c1), 0, 150, 0, 100);
        correction2 = map(abs(c2), 0, 250, 0, 100);

        if (error1 < 0 )
        {
          upr_mtr_fwd();
          analogWrite(motor1pwm, abs(correction1));
        }
        else if (error1 > 0)
        {
          upr_mtr_bwd();
          analogWrite(motor1pwm, abs(correction1));
        }

        if (error2 < 0)
        {
          lwr_mtr_fwd();
          analogWrite(motor2pwm, abs(correction2));
        }
        else if (error2 > 0)
        {
          lwr_mtr_bwd();
          analogWrite(motor2pwm, abs(correction2));
        }
      }
    }
  }
}

//------------------------------------------------------------------------------F U N C T I O N S-----------------------------------------------------------------------------------------

float cosine_rule(float c, float b, float a)
{
  float x = ( a * a + b * b - c * c ) / ( 2 * a * b );
  return acos(x);
}

void upr_mtr_fwd()
{
  digitalWrite(motor1, HIGH);
}

void upr_mtr_bwd()
{
  digitalWrite(motor1, LOW);
}

void lwr_mtr_fwd()
{
  digitalWrite(motor2, HIGH);
}

void lwr_mtr_bwd()
{
  digitalWrite(motor2, LOW);
}

void ai0()
{
  if (digitalRead(3) == LOW)
  {
    counter1++;
  } else {
    counter1--;
  }
}

void ai1() {
  if (digitalRead(2) == LOW)
  {
    counter1--;
  } else {
    counter1++;
  }
}

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
