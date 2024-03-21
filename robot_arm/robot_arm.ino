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

#define SERVO_MIN 100
#define SERVO_MAX 660

#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0; // initialize servo counter


// --- PIN DEFINITIONS ---

// --- VARIABLE DEFINITIONS ---

// Define other variables
int pwm;
const int num_servos = 5;
unsigned long time_0;
unsigned long new_time;
int timer;
int start_time;
int end_time;

// --- PWM PARAMETERS ---
const int target_pwm_tolerance = 1;
const int pwm_step = 1;
const int step_delay = 10;
int current_angle[5] = {90, 90, 90, 90, 90};
int current_pwm[5] = {395, 395, 395, 395, 395};
int pwm_increments[5] = {2, 1.67222, 1.7055, 1.7889, 2};

// --- PHYSICAL JOINT PARAMETERS ---
const float joint_max_angles[5] = {270.0, 270.0, 270.0, 270.0, 180.0};
const int joint_min_pwm[5] = {120, 161, 153, 136, 130}; // CALIBRATED 03/10/2024
const int joint_max_pwm[5] = {660, 613, 613, 619, 650}; // CALIBRATED 03/10/2024

// --- ALLOWABLE JOINT PARAMETERS ---
const float joint_min_allowable_angles[5] = {45, 45, 45, 45, 10}; // TBD
const float joint_max_allowable_angles[5] {250, 250, 250, 250, 170};  // TBD

// --- JOINT VARIABLES ---
float joint_angles[5] = {135.0, 135.0, 135.0, 135.0, 90.0};
int joint_pwm[5] = {390, 395, 395, 395, 390}; // TBD


// --- ZERO VALUES ---
const float zero_angles[5] = {135.0, 135.0, 135.0, 135.0, 90.0};
const int zero_pos_pwm[5] = {390, 388, 383, 377, 390};


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

  time_0 = millis();

  // Set servos to initial position
  // Arguments of setPWM are (servo number, turn on, turn off)
  // Turn on: At what point in the 4096-part cycle to turn the PWM output ON
  // Turn off: At what point in the 4096-part cycle to turn the PWM output OFF
  // force_vertical();
  // servo_calibration();
  // delay(500);
  timer = millis();
}


// --- MAIN LOOP ---

void loop() {
  serial_to_joint_angles();
  // serial_read_test();
}


// --- SERVO FUNCTIONS ---

void servo_calibration() {
  servo.setPWM(0, 0, 387);
}


void step_all_to_position(int target_angle) {
  int target_pwm;
  int pwm_diff[5];
  int abs_pwm_diff[5];

  target_reached = false;
  target_pwm = angle_to_pwm(target_angle, 0, 180, SERVO_MIN, SERVO_MAX);

  while (target_reached != true)  {
    for (int i=0; i<num_servos; i++)  {

      pwm_diff[i] = target_pwm - joint_pwm[i];
      abs_pwm_diff[i] = abs(pwm_diff[i]);

      if ((pwm_diff[i] > 0) && (abs_pwm_diff[i] > target_pwm_tolerance)) {
        servo.setPWM(i, 0, joint_pwm[i]);
        joint_pwm[i]+=pwm_step;
      } else if ((pwm_diff[i] < 0) && (abs_pwm_diff[i] > target_pwm_tolerance))  {
        servo.setPWM(i, 0, joint_pwm[i]);
        joint_pwm[i]-=pwm_step;
      } else if (abs_pwm_diff[i] <= target_pwm_tolerance)  {
        target_reached = true;
      }

      current_angle[i] = pwm_to_angle(joint_pwm[i], SERVO_MIN, SERVO_MAX, 0, 180);

      delay(step_delay);
    }
  }
}


// --- PRE-DEFINED POSITIONS ---

// Change to an actual home position
void home() {
  for (int i=1; i<num_servos; i++) {
    servo.setPWM(i, 0, 330);
  }
}

void force_vertical() {
  // for (int i=0; i<num_servos; i++) {
  //   servo.setPWM(i, 0, zero_pos_pwm[i]);
  //   joint_pwm[i] = zero_pos_pwm[i];
  float mutable_zero[5] = {zero_angles[0], zero_angles[1], zero_angles[2], zero_angles[3], zero_angles[4]};
  set_joint_angles(mutable_zero);
  // }
}

// Useless function?
void step_to_vertical() {
  float vertical_targets[5] = {135, 135, 90, 90, 90};
  set_joint_angles(vertical_targets);
}

// --- UTILITY FUNCTIONS ---
int pwm_to_angle(int internal_pwm, int min_pwm, int max_pwm, int min_angle, int max_angle) {
  int angle_output;

  angle_output = map(internal_pwm, min_pwm, max_pwm, min_angle, max_angle);

  return angle_output;
}


int angle_to_pwm(int angle_input, int min_angle, int max_angle, int min_pwm, int max_pwm) {
  int pwm_output;

  pwm_output = map(angle_input, min_angle, max_angle, min_pwm, max_pwm);

  return pwm_output;
}


//  --- TEST FUNCTIONS ---

void set_joint_angles(float joint_angle_targets[5]) {
  start_time = millis();
  Serial.print("start: ");
  Serial.println(start_time);

  for (int i=0; i<num_servos; i++)  {
    joint_target_reached[i] = false;
  }

  float joint_pwm_targets[5];
  float joint_pwm_diffs[5];

  int target_counter=0;

  timer = millis();
  Serial.print("while start: ");
  Serial.println(timer);
  while (target_counter < num_servos)  {
    for (int j=0; j<num_servos; j++)  {

      target_counter=0;

      // Check each joint to see if the target has been reached
      for (int k=0; k<num_servos; k++)  {
        if (joint_target_reached[k]==true) {
          target_counter++;
        }
      }
      
      joint_pwm_targets[j] = map(joint_angle_targets[j], 0, joint_max_angles[j], joint_min_pwm[j], joint_max_pwm[j]);
      // Serial.println(joint_pwm_targets[j]);

      joint_pwm_diffs[j] = joint_pwm_targets[j] - joint_pwm[j];

      timer = millis();
      Serial.print("if start: ");
      Serial.println(timer);
      if (joint_pwm_diffs[j] > target_pwm_tolerance) {
        servo.setPWM(j, 0, joint_pwm[j]);
        joint_pwm[j] += pwm_step;
      } else if (joint_pwm_diffs[j] < -(target_pwm_tolerance))  {
        servo.setPWM(j, 0, joint_pwm[j]);
        joint_pwm[j] -= pwm_step;
      } else  {
        joint_target_reached[j] = true;
      }

      timer = millis();
      Serial.print("if end: ");
      Serial.println(timer);
    }
    delay(step_delay);
  }
  end_time = millis();
  Serial.print("end: ");
  Serial.println(end_time);
}


void serial_read_test() {
  if (Serial.available() > 5) {
    Serial.println(Serial.readBytesUntil(END_MARKER, incoming_buffer, 30));
  }
}


void serial_to_joint_angles() {
  if (Serial.available() > 0)  {
    
    incoming_byte = Serial.readBytesUntil(END_MARKER, incoming_buffer, 30);
    Serial.read();  // Clear input buffer
    Serial.println(incoming_byte);

    char *token = strtok(incoming_buffer, DELIMITER);
    int tok_counter = 0;
    
    while(token != NULL)  {
      float fl_tok = atof(token);
      joint_angles[tok_counter] = fl_tok;
      token = strtok(NULL, DELIMITER);
      tok_counter++;
    }

    for (int i=0; i<(num_servos-1); i++)  {
      if(i==1)  {
        joint_angles[i] = zero_angles[i] - joint_angles[i]; // This servo is mounted in reverse
      } else {
        joint_angles[i] = zero_angles[i] + joint_angles[i];
      }
    }

    set_joint_angles(joint_angles);
  }
}