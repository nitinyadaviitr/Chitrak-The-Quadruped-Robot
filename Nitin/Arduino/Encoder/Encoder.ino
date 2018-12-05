volatile int temp, counter = 0; //This variable will increase or decrease depending on the rotation of encoder
int a = 4;
int b = 3;  
void setup() {
  Serial.begin (9600);
  pinMode(a, INPUT_PULLUP);
  pinMode(b, INPUT_PULLUP);
  attachInterrupt(1, ai0, RISING);
  }
   
  void loop() {
  if( counter != temp ){
  temp = counter;
  if(counter > 600){
    counter = 0;
    }
    
    float angle = (counter*0.6);
    
    Serial.println(angle);
  }
  }
   
  void ai0() {
  if(digitalRead(a)==LOW) {
  counter++;
  }else{
  counter--;
  }
  }
  
