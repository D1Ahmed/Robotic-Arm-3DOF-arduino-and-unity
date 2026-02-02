#include <Servo.h>
Servo baseServo;     
Servo shoulderServo; 
Servo elbowServo;    
void setup() {
  baseServo.attach(9);
  shoulderServo.attach(10);
  elbowServo.attach(11);
  Serial.begin(9600);
}
void loop() {
  int a1 = map(analogRead(A0), 0, 1023, 0, 180);
  int a2 = map(analogRead(A1), 0, 1023, 0, 180);
  int a3 = map(analogRead(A2), 0, 1023, 0, 180);
  if (Serial.available() > 0) {
    int s1 = Serial.parseInt();
    int s2 = Serial.parseInt();
    int s3 = Serial.parseInt();
    
    while(Serial.available() > 0) Serial.read();
    a1 = s1;
    a2 = s2;
    a3 = s3;
  }
  baseServo.write(a1);
  shoulderServo.write(a2);
  elbowServo.write(a3);
  Serial.print(a1); Serial.print(",");
  Serial.print(a2); Serial.print(",");
  Serial.println(a3);

  delay(20); 
}