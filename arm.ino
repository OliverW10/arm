#include <Servo.h>


Servo servoYaw;
Servo servoPitch;
Servo servoWrist;
String message;

void setup(){
    Serial.begin(9600);
    servoYaw.attach(4);
    servoPitch.attach(5);
    servoWrist.attach(6);
}

void loop(){
  if (Serial.available() > 5) {
    int cur = 0;
    message = Serial.readStringUntil(';');
    byte angleYaw = message.substring(cur, message.indexOf(",", cur)).toInt();
    cur = message.indexOf(",") + 1;
    byte anglePitch = message.substring(cur, message.indexOf(",", cur)).toInt();
    cur = message.indexOf(",") + 1;
    byte angleWrist = message.substring(cur, message.indexOf(",", cur)).toInt();
    cur = message.indexOf(",") + 1;
    if(!(angleYaw == 0 && anglePitch == 0 && angleWrist == 0)){
      servoYaw.write(angleYaw);
      servoPitch.write(anglePitch);
      servoWrist.write(angleWrist);
      
//      Serial.print("Got angles: ");
//      Serial.print(angleYaw, DEC);
//      Serial.print(", ");
//      Serial.print(anglePitch, DEC);
//      Serial.print(", ");
//      Serial.println(angleWrist, DEC);
    }
  }
}
