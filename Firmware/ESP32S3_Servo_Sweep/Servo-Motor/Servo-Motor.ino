#include <s3servo.h>
s3servo servo1;

//you must have esp driver version 2.0(search when ESP32-S3 was released) in order for 
//the servo library to work

//!! DO NOT UPDATE THE LIBRARY !!

//pin on the controller where the servo is connected
int servoPin = 46;   

//setup
void setup() {
    servo1.attach(servoPin);
}

//two for loops that rotate the motor from 30 to 150 degrees(first loop) 
//and from 150 to 30 degrees(second loop)
void loop() {
    for(int i = 30; i <= 150; i++) {
        servo1.write(i);
        delay(22);
    }
    for(int i = 150; i >= 30; i--) {
        servo1.write(i);
        delay(22);
    }
}
