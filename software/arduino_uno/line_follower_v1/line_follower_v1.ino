// #include "motor.h"
// #include "line_process.h"
// #include "PID_control.h"

#define line_left_2 0
#define line_left_1 1
#define line_center 2
#define line_right_1 3
#define line_right_2 4
// #define encoder_right 12
// #define encoder_left 7
#define pwm_motor_fr 10
#define pwm_motor_br 9
#define pwm_motor_fl 3 
#define pwm_motor_bl 11
#define MOTOR_FR OCR2B
#define MOTOR_BR OCR2A
#define MOTOR_FL OCR1B
#define MOTOR_BL OCR1A
enum TURN_STATE {
  STRAIGHT = 0,
  EASY,
  NORMAL,
  HARD,
};

/*  Motor variables  */
const float scale = 7.0;
uint16_t base_sp = 100;
int delta, sum;
int turn_state = STRAIGHT;

/*  PID variables   */
const float Kp = 3.70;
const float Kd = 0.35;
const float desired_pos = 0.0;
float error, derivative, output, prev_error, dt;

/*  Line detecter variables  */
int line_left_1_pos, line_left_2_pos, line_right_1_pos, line_right_2_pos, line_center_pos, sum_pos;
float current_pos, prev_pos;
int times = 0;

/*  Encoder variables  */
// unsigned long encoder_left_count = 0, encoder_right_count = 0;

/*  MyDelay variables  */
unsigned long last_time = 0;
unsigned long last_PID_loop_time;

bool my_delay(uint32_t time) {
  unsigned long now = micros();
  if (now - last_time > time) {
    last_time = now;
    return true;
  }
  return false;
}

bool readDDC(uint8_t pin) {
  return (PINC & (1 << pin));
}

void PID_control() {
  error = desired_pos - current_pos;
  unsigned long now = micros();
  dt = (now - last_PID_loop_time) / 1000.0;
  if (dt <= 0) dt = 0.002;
  last_PID_loop_time = now;
  derivative = (error - prev_error) / dt;
  output = Kp*error + Kd*derivative;
  if (abs(error) == 0) {
    turn_state = STRAIGHT;
  } else if (abs(error) >= 3.0) {
    output *= 1.75;
    turn_state = HARD;
  } else if (abs(error) >= 2.25) {
    output *= 1.25;
    turn_state = NORMAL;
  } else {
    turn_state = EASY;
  }
  prev_error = error;
}

void motor_control() {
  delta = constrain(output * scale, -165, 165);

  if (turn_state == STRAIGHT) {
    // Go straight
    MOTOR_FL = base_sp;
    MOTOR_BL = 0;
    MOTOR_FR = base_sp;
    MOTOR_BR = 0;
  } else if (turn_state == EASY || turn_state == NORMAL) {
    MOTOR_FL = constrain(base_sp - delta, 0, 255);;
    MOTOR_BL = 0;
    MOTOR_FR = constrain(base_sp + delta, 0, 255);
    MOTOR_BR = 0;
  } else if (turn_state == HARD) {
    if (output > 0) {
      // Turn right
      MOTOR_FL = 0;
      MOTOR_BL = constrain(base_sp + abs(delta)/2, 0, 255);
      MOTOR_FR = constrain(base_sp + abs(delta), 0, 255);
      MOTOR_BR = 0;
    } else if (output < 0) {
      // Turn left
      MOTOR_FL = constrain(base_sp + abs(delta), 0, 255);
      MOTOR_BL = 0;
      MOTOR_FR = 0;
      MOTOR_BR = constrain(base_sp + abs(delta)/2, 0, 255);
    }
  }
}

void get_current_position() {
  line_left_2_pos = readDDC(line_left_2);
  line_left_1_pos = readDDC(line_left_1);
  line_center_pos = readDDC(line_center);
  line_right_1_pos = readDDC(line_right_1);
  line_right_2_pos = readDDC(line_right_2);

  sum_pos = line_left_2_pos + line_left_1_pos + line_center_pos + line_right_1_pos + line_right_2_pos;

  if (sum_pos > 0) {
    current_pos = ((line_left_2_pos * (-3.0)) + (line_left_1_pos * (-1.5)) + (line_center_pos * 0.0) + (line_right_1_pos * 1.5) + (line_right_2_pos * 3.0))/(float)sum_pos;
    prev_pos = current_pos;
    times = 0;
  } else {
    //Control exception
    current_pos = prev_pos;
    times++;
    if (times > 250) {
      current_pos += 0.1;
    } 
  }
}

void car_stop() {
  MOTOR_BL = 0;
  MOTOR_BR = 0;
  MOTOR_FL = 0;
  MOTOR_FR = 0;
}

void set_timer() {
  // --- Timer1 (pin 9, 10) = LEFT ---
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);  // Fast PWM 8-bit
  TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);  // Prescaler = 64 → 250kHz

  // --- Timer2 (pin 3, 11) = RIGHT ---
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);  // Fast PWM 8-bit
  TCCR2B = _BV(CS22) | _BV(CS20);  // Prescaler = 128 → 250kHz
}

void motor_init() {
  pinMode(pwm_motor_fl, OUTPUT);
  pinMode(pwm_motor_fr, OUTPUT);
  pinMode(pwm_motor_bl, OUTPUT);
  pinMode(pwm_motor_br, OUTPUT);
  // pinMode(encoder_left, INPUT);
  // pinMode(encoder_right, INPUT);

  set_timer();
}

void line_detecter_init() {
  pinMode(line_left_2, INPUT);
  pinMode(line_left_1, INPUT);
  pinMode(line_center, INPUT);
  pinMode(line_right_1, INPUT);
  pinMode(line_right_2, INPUT);
}

void setup() {
  // Serial.begin(115200);
  line_detecter_init();
  motor_init();
  delay(1000);
}

void loop() {
  if(my_delay(2000)) {
    get_current_position();
    PID_control();
    motor_control();
    // Serial.print(line_left_2_pos);
    // Serial.print(" - ");
    // Serial.print(line_left_1_pos);
    // Serial.print(" - ");
    // Serial.print(line_center_pos);
    // Serial.print(" - ");
    // Serial.print(line_right_1_pos);
    // Serial.print(" - ");
    // Serial.println(line_right_2_pos);
    // Serial.println(current_pos);
    // Serial.print(" - ");
    // Serial.println(output);
  }
}
