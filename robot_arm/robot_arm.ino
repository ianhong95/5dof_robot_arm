// #include <PCA9685.h>
// PCA9685 pwmController;

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27,20,4);
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(0x40);

// 0: 120-660, 395
// 1: 95-680, 391
// 3: 130-650, 390

#define SERVO_1_MIN  100 // This is the 'minimum' pulse length count (out of 4096)
#define SERVO_1_HOME  388
#define SERVO_1_MAX  675 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_2_MIN  135 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_2_HOME  393
#define SERVO_2_MAX  650 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_3_MIN  95 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_3_MAX  680 // This is the 'maximum' pulse length count (out of 4096)

#define TEST_SERVO_MIN 80
#define TEST_SERVO_HOME 400
#define TEST_SERVO_MAX 750
// #define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
// #define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 60 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0; // initialize servo counter


// --- PIN DEFINITIONS ---

const int dig_pin_2 = 2;
const int dig_pin_4 = 4;

const int ana_pin_0 = 14;
const int ana_pin_1 = 15;

const int pwm_pin_5 = 5;
const int pwm_pin_6 = 6;

// --- VARIABLE DEFINITIONS ---

// Define indices
int current_servo_idx=0;

// Define reading variables

int raw_pot_reading;
int pot_reading_0;

// Define other variables
int pwm_0 = TEST_SERVO_HOME;
int pwm;
const int num_servos = 5;
unsigned long time_0;
unsigned long new_time;

// --- PWM PARAMETERS ---
const int pwm_deadband = 5;
const int angle_deadband = 1;
const int target_pwm_tolerance = 1;
const int pwm_step = 1;
const int step_delay = 10;
int current_angle[5] = {90, 90, 90, 90, 90};
int current_pwm[5] = {395, 395, 395, 395, 395};

// --- PHYSICAL JOINT PARAMETERS ---
const float joint_max_angles[5] = {270.0, 270.0, 180.0, 180.0, 180.0};
const int joint_min_pwm[5] = {120, 100, 130, 150, 130}; // CALIBRATED
const int joint_max_pwm[5] = {660, 685, 650, 650, 650}; // CALIBRATED
const int joint_home_pwm[5] = {390, 390, 390, 390, 390};


// --- ALLOWABLE JOINT PARAMETERS ---
const float joint_min_allowable_angles[5] = {45, 90, 30, 30, 30}; // TBD
const float joint_max_allowable_angles[5] {215, 150, 135, 135, 135};  // TBD

// --- JOINT VARIABLES ---
float joint_angles[5] = {135.0, 135.0, 90.0, 90.0, 90.0};
int joint_pwm[5] = {390, 390, 390, 390, 390}; // TBD


// --- CHECK VARIABLES ---
bool target_reached;
bool joint_target_reached[5] = {false, false, false, false, false};


// --- SETUP ---

void setup() {

  // Start listeners
  Serial.begin(9600);
  servo.begin();
  lcd.init();

  // Set PWM frequency
  servo.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // Set up pins
  pinMode(dig_pin_2, OUTPUT);
  pinMode(dig_pin_4, INPUT);
  pinMode(pwm_pin_5, OUTPUT);
  pinMode(pwm_pin_6, OUTPUT);
  pinMode(ana_pin_0, INPUT);
  pinMode(ana_pin_1, INPUT);

  // Set up lcd
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("PWM: ");
  lcd.setCursor(0, 1);
  lcd.print("Servo: 0");

  time_0 = millis();

  // Initialize potentiometer readings
  pot_reading_0 = analogRead(ana_pin_0);
  pwm_0 = pot_to_pwm(pot_reading_0, TEST_SERVO_MIN, TEST_SERVO_MAX);

  // Set servos to initial position
  // Arguments of setPWM are (servo number, turn on, turn off)
  // Turn on: At what point in the 4096-part cycle to turn the PWM output ON
  // Turn off: At what point in the 4096-part cycle to turn the PWM output OFF
  force_vertical();
  delay(1000);

  // step_all_to_position(60);
  // delay(1000);

  float test_targets[5] = {135, 135, 90, 90, 90};
  float test_targets_2[5] = {120, 120, 60, 60, 60};

  set_joint_angles(test_targets_2);
  delay(1000);

  set_joint_angles(test_targets);
  delay(1000);

  set_joint_angles(test_targets_2);
  delay(1000);

  step_to_vertical();
  delay(1000);

}


// --- MAIN LOOP ---

void loop() {
  check_main_btn(dig_pin_2);

  raw_pot_reading = analogRead(ana_pin_0);
  pwm = pot_to_pwm(ana_pin_0, TEST_SERVO_MIN, TEST_SERVO_MAX);

  // --- FOR CALIBRATION ---
  // Serial.println(pwm);
  // servo.setPWM(5, 0, pwm);
  // -------------------

  new_time = millis();

  // Only print to LCD ever 100ms and if raw pot reading changes significantly
  if ((new_time - time_0 > 100) && (abs(raw_pot_reading - pot_reading_0) > 5))  {
    lcd_clear_section(0, 5, 8);
    lcd.setCursor(5, 0);
    lcd.print(pwm);
    time_0 = new_time;
  }

  int pwm_reading;

  // if (abs(raw_pot_reading-pot_reading_0) > 5) {
  //   pwm_reading = servo.getPWM(0, false);
  //   pot_reading_0 = raw_pot_reading;
  //   pwm_0 = pwm;
  // }
}


// --- SERVO FUNCTIONS ---
void step_all_to_position(int target_angle) {
  int target_pwm;
  int pwm_diff[5];
  int abs_pwm_diff[5];

  target_reached = false;
  target_pwm = angle_to_pwm(target_angle, 0, 180, TEST_SERVO_MIN, TEST_SERVO_MAX);

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

      current_angle[i] = pwm_to_angle(joint_pwm[i], TEST_SERVO_MIN, TEST_SERVO_MAX, 0, 180);

      delay(step_delay);
    }
  }
}


// --- LCD FUNCTIONS ---

void lcd_clear_section(int row, int start_char, int end_char) {
  for (int i=start_char; i<(end_char+1); i++)  {
    lcd.setCursor(i, row);
    lcd.print(" ");
  }
}


void lcd_clear_row(int row) {
  lcd.setCursor(0, row);
  lcd.print("                ");
}


// --- BUTTON PRESS FUNCTIONS ---

// If the main button is pressed, change the motor
void check_main_btn(int main_btn_pin) {
  int main_btn_reading;
  main_btn_reading = digitalRead(main_btn_pin);

  if (main_btn_reading==HIGH) {
    // set_servo_idx();
    // lcd_clear_section(1, 7, 8);
    // lcd.setCursor(7, 1);
    // lcd.print(current_servo_idx);
    // delay(500);
    step_to_vertical();
  }
}


// --- PRE-DEFINED POSITIONS ---

void home() {
  for (int i=1; i<num_servos; i++) {
    servo.setPWM(i, 0, 330);
  }
}


void force_vertical() {
  for (int i=0; i<num_servos; i++) {
    servo.setPWM(i, 0, TEST_SERVO_HOME);
    joint_pwm[i] = 390;
  }
}


void step_to_vertical() {
  float vertical_targets[5] = {135, 135, 90, 90, 90};
  set_joint_angles(vertical_targets);
}

// --- UTILITY FUNCTIONS ---

int pot_to_pwm(int pot_pin, int servo_min, int servo_max)  {
  int pwm_output;

  raw_pot_reading = analogRead(pot_pin);
  pwm_output = map(raw_pot_reading, 0, 1023, servo_min, servo_max);
  
  return pwm_output;
}


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


void set_servo_idx()  {
  if (current_servo_idx < (num_servos - 1))  {
    current_servo_idx += 1;
  } else if (current_servo_idx == (num_servos - 1)) {
    current_servo_idx = 0;
  } else {
    current_servo_idx = 0;
    Serial.println("Invalid servo index!");
  }
}


//  --- TEST FUNCTIONS ---

void set_joint_angles(float joint_angle_targets[5]) {
  for (int i=0; i<num_servos; i++)  {
    joint_target_reached[i] = false;
  }

  int joint_pwm_targets[5];
  int joint_pwm_diffs[5];

  int target_counter=0;

  while (target_counter < num_servos)  {
    for (int j=0; j<num_servos; j++)  {

      target_counter=0;
      for (int k=0; k<num_servos; k++)  {
        if (joint_target_reached[k]==true) {
          target_counter++;
        }
      }
      
      joint_pwm_targets[j] = map(joint_angle_targets[j], 0, joint_max_angles[j], joint_min_pwm[j], joint_max_pwm[j]);
      Serial.println(joint_pwm_targets[j]);

      joint_pwm_diffs[j] = joint_pwm_targets[j] - joint_pwm[j];

      if (joint_pwm_diffs[j] > target_pwm_tolerance) {
        servo.setPWM(j, 0, joint_pwm[j]);
        joint_pwm[j] += pwm_step;
      } else if (joint_pwm_diffs[j] < -(target_pwm_tolerance))  {
        servo.setPWM(j, 0, joint_pwm[j]);
        joint_pwm[j] -= pwm_step;
      } else  {
        joint_target_reached[j] = true;
      }
    }
    delay(step_delay);
  }
}