#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN 100
#define SERVO_MAX 660

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0; // initialize servo counter

void setup() {
    servo.setPWM(3, 0, 390);
}

void loop() {
}
