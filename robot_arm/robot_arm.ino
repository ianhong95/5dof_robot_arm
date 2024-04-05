// Standard C libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Standard Arduino libraries
#include <Wire.h>

// 3rd party Arduino libraries
// #include <PCA9685.h>
// PCA9685 pwmController;
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x40);

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0; // initialize servo counter


// --- PIN DEFINITIONS ---

// --- VARIABLE DEFINITIONS ---

// Define other variables
int pwm;
const int num_servos = 6;
unsigned long time_0;
unsigned long new_time;
int timer;
int start_time;
int end_time;
const int GRIPPER_SERVO_IDX = 5;

// --- PWM PARAMETERS ---
const int target_pwm_tolerance = 1;
const int pwm_step = 1;
const int step_delay = 5;
float pwm_increments[5] = {2, 1.74817482, 1.7055, 1.7889, 2};  // pwm increase per degree of rotation
float joint_pwm_targets[5];
const float GRIPPER_MIN_PWM = 130;  // TBD
const float GRIPPER_MAX_PWM = 385;  // TBD


// --- PHYSICAL JOINT PARAMETERS ---
const float joint_max_angles[5] = {270.0, 270.0, 270.0, 270.0, 180.0};
const float joint_min_pwm[5] = {141.0, 153.0, 146.0, 135.0, 130.0}; // CALIBRATED 03/24/2024 using jig
const float joint_max_pwm[5] = {640.0, 625.0, 620.0, 616.0, 650.0}; // CALIBRATED 03/24/2024 using jig

// --- ALLOWABLE JOINT PARAMETERS ---
const float joint_min_allowable_angles[5] = {45, 45, 45, 45, 10}; // TBD
const float joint_max_allowable_angles[5] {250, 250, 250, 250, 170};  // TBD

// --- JOINT VARIABLES ---
float joint_angles[5] = {135.0, 135.0, 135.0, 135.0, 90.0};
int gripper_state_buffer[2];
float joint_pwm[5] = {398.0, 387.0, 385.0, 381.0, 390.0};
float adjusted_joint_angles[5]; // Adjusted based on zero positions and servo orientation


// --- ZERO VALUES ---
const float zero_angles[5] = {135.0, 135.0, 135.0, 135.0, 90.0};
const float zero_pos_pwm[5] = {398.0, 387.0, 385.0, 381.0, 390.0};


// --- CHECK VARIABLES ---
bool target_reached;
bool joint_target_reached[5] = {false, false, false, false, false};


// --- SERIAL ---
char START_MARKER = '<';
char END_MARKER = '>';
char DELIMITER[] = ",";
int incoming_byte;
char incoming_buffer[40];
char *incoming_joint_angles[5];


// --- SETUP ---

void setup() {

  // Start listeners
  Serial.begin(115200);
  servo.begin();

  // Set PWM frequency
  servo.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // Set up pins

  // Set servos to initial position
  // Arguments of setPWM are (servo number, turn on, turn off)
  // Turn on: At what point in the 4096-part cycle to turn the PWM output ON
  // Turn off: At what point in the 4096-part cycle to turn the PWM output OFF
  force_vertical();
  // set_zero_pwm();
  // servo_calibration();
  delay(500);
  timer = millis();
}


// --- MAIN LOOP ---

void loop() {
  serial_to_joint_angles();
  // serial_read_test();
}


// --- SERVO FUNCTIONS ---

void servo_calibration() {

  servo.setPWM(4, 0, 130);
  delay(2000);
  servo.setPWM(4, 0, 385);
  delay(3000);
  servo.setPWM(4, 0, 130);
  delay(1000);
  // servo.setPWM(0, 0, 154);
  // delay(5000);
  // servo.setPWM(0, 0, 387);

  // int zero = map(135, 0, 270, 154, 626);
  // int min = map(0, 0, 270, 154, 626);
  // int max = map(270, 0, 270, 154, 626);
  // Serial.println(zero);
  // servo.setPWM(0, 0, zero);
  // delay(5000);
  // servo.setPWM(0, 0, min);
  // delay(5000);
  // servo.setPWM(0, 0, max);
  // delay(5000);
  // servo.setPWM(0, 0, zero);
}


// --- PRE-DEFINED POSITIONS ---

// Change to an actual home position
void home() {
  for (int i=1; i<num_servos; i++) {
    servo.setPWM(i, 0, 330);
  }
}


void set_zero_pwm() {
  for (int i=0; i<num_servos; i++) {
    servo.setPWM(i, 0, zero_pos_pwm[i]);
    joint_pwm[i] = zero_pos_pwm[i];
  }

}

void force_vertical() {
  // for (int i=0; i<num_servos; i++) {
  //   servo.setPWM(i, 0, zero_pos_pwm[i]);
  //   joint_pwm[i] = zero_pos_pwm[i];
  float mutable_zero[5] = {zero_angles[0], zero_angles[1], zero_angles[2], zero_angles[3], zero_angles[4]};
  step_joint_angles(mutable_zero);
  // }
}

// --- UTILITY FUNCTIONS ---


//  --- TEST FUNCTIONS ---

void step_joint_angles(float joint_angle_targets[5]) {
  float joint_pwm_diffs[5];

  for (int i=0; i<num_servos-1; i++)  {
    joint_target_reached[i] = false;
    joint_pwm_targets[i] = map(joint_angle_targets[i], 0, joint_max_angles[i], joint_min_pwm[i], joint_max_pwm[i]);
  }

  while (joint_target_reached[0]==false || joint_target_reached[1]==false || joint_target_reached[2]==false || joint_target_reached[3]==false)  {
    for (int j=0; j<num_servos-1; j++)  {

      joint_pwm_diffs[j] = joint_pwm_targets[j] - joint_pwm[j];

      if ((joint_pwm_diffs[j] > target_pwm_tolerance) && (joint_target_reached[j]!=true)) {
        joint_pwm[j] += pwm_step;
        servo.setPWM(j, 0, joint_pwm[j]);
      } else if (joint_pwm_diffs[j] < -(target_pwm_tolerance) && (joint_target_reached[j]!=true))  {
        joint_pwm[j] -= pwm_step;
        servo.setPWM(j, 0, joint_pwm[j]);
      } else  {
        joint_target_reached[j] = true;
      }
    }
    delay(step_delay);
  }
}


void go_to_position(float joint_angle_targets[5]) {
  for (int i=0; i<num_servos; i++)  {
    joint_pwm_targets[i] = map(joint_angle_targets[i], 0, joint_max_angles[i], joint_min_pwm[i], joint_max_pwm[i]);
    servo.setPWM(i, 0, joint_pwm_targets[i]);
  }
}


void serial_read_test() {
  if (Serial.available() > 5) {
    Serial.println(Serial.readBytesUntil(END_MARKER, incoming_buffer, 30));
  }
}

// General function for handling incoming serial data
void serial_to_joint_angles() {
  if (Serial.available() > 0)  {
    
    incoming_byte = Serial.readBytesUntil(END_MARKER, incoming_buffer, 50);
    Serial.read();  // Clear input buffer
    Serial.println("Data received!");

    if (incoming_buffer[0] == START_MARKER) {
      set_gripper(incoming_buffer[2]);
    } else {
      // Parse the incoming data and split it along the delimiters, then build a list of values
      // First call of strtok takes the initial string. The first non-leading delimiter is replaced with a NULL character.
      char *token = strtok(incoming_buffer, DELIMITER);
      int tok_counter = 0;
      
      while(token != NULL)  {
        float fl_tok = atof(token); // Initialize a float, assign it a value using atof based on the token (index)
        joint_angles[tok_counter] = fl_tok; // Put a float into the joint_angles[] array

        // When the strtok() function is called with a NULL string1 argument, the next token is read from a stored copy of the last non-null string1 parameter. Each delimiter is replaced by a null character.
        token = strtok(NULL, DELIMITER);
        tok_counter++;
      }
      // End parsing

      Serial.println(joint_angles[0]);
      Serial.println(joint_angles[1]);
      Serial.println(joint_angles[2]);
      Serial.println(joint_angles[3]);

      // Move joints
      for (int i=0; i<(num_servos-1); i++)  {
        if(i==1)  {
          adjusted_joint_angles[i] = zero_angles[i] - joint_angles[i]; // This servo is mounted in reverse
        } else {
          adjusted_joint_angles[i] = zero_angles[i] + joint_angles[i];
        }
      }
      
      step_joint_angles(adjusted_joint_angles);
    }
  }
}


void set_gripper(char gripper_pos)  {
  if (gripper_pos=='0')  {
    servo.setPWM(GRIPPER_SERVO_IDX, 0, 385);
  } else if (gripper_pos=='1')  {
    servo.setPWM(GRIPPER_SERVO_IDX, 0, 130);
  }
}