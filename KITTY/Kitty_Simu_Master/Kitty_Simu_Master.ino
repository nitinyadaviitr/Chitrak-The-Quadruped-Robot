#include <SoftwareSerial.h>

SoftwareSerial mySerial1(3, 4);
SoftwareSerial mySerial2(5, 6);
SoftwareSerial mySerial3(7, 8);
SoftwareSerial mySerial4(9, 10);

void setup() {
  
  mySerial1.begin(115200);
  mySerial2.begin(115200);
  mySerial3.begin(115200);
  mySerial4.begin(115200);
}

void loop() {

  mySerial1.write(1);
  
}
