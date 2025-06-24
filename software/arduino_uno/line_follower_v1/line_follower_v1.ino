#define line_left A0
#define line_center A1
#define line_right A2
#define encoder_right 12
#define encoder_left 7
#define pwm_motor_fl 5
#define pwm_motor_fr 3
#define pwm_motor_bl 9
#define pwm_motor_br 6 

/*  PID variables   */
const float Kp = 2.8;
const float Ki = 0.15;
const float Kd = 0.02u77i9oo;
const float desired_pos = 0.0;
float error, derivative, output, prev_error, dt;
float integral = 0.0;
uint32_t time_reset = 10000;

/*  Line detecter variables  */
int line_left_pos, line_right_pos, line_center_pos;
float current_pos, prev_pos;

/*  Encoder variables  */
unsigned long encoder_left_count = 0, encoder_right_count = 0;

/*  MyDelay variables  */
unsigned long last_time = 0;
unsigned long last_time_reset, last_PID_loop_time;

/*  Motor variables  */
const float scale = 22.2;
uint16_t base_sp = 90;
int delta;

bool my_delay(uint32_t time) {
  if (millis() - last_time > time) {
    last_time = millis();
    return true;
  }
  return false;
}

void PID_control() {
  error = desired_pos - current_pos;
  unsigned long now = millis();
  dt = (now - last_PID_loop_time) / 1000.0;
  if (dt <= 0) dt = 0.01;
  last_PID_loop_time = now;
  if (millis() - last_time_reset > time_reset) {
    last_time_reset = millis();
    integral = 0;
  } else {
    integral += (error * dt);
  }
  derivative = (error - prev_error) / dt;
  output = Kp*error + Ki*integral + Kd*derivative;
  // if (abs(error) >= 1.0) {
  //   output *= 1.5;
  // }
  prev_error = error;
}

void motor_control() {
  delta = constrain(output * scale, -100, 100);
  analogWrite(pwm_motor_fl, constrain(base_sp - delta, 0, 255));
  analogWrite(pwm_motor_fr, constrain(base_sp + delta, 0, 255));
  analogWrite(pwm_motor_bl, constrain(base_sp - delta, 0, 255));
  analogWrite(pwm_motor_br, constrain(base_sp + delta, 0, 255));
}

void get_current_position() {
  line_left_pos = analogRead(line_left) + 70;
  line_center_pos = analogRead(line_center);
  line_right_pos = analogRead(line_right);

  uint16_t sum = line_left_pos + line_center_pos + line_right_pos;

  if (sum > 0) {
    current_pos = ((line_left_pos * (-2.0)) + (line_center_pos * 0.0) + (line_right_pos * 2.0))/(float)sum;
    prev_pos = current_pos;
  } else {
    //Control exception
    current_pos = prev_pos;
  }
}

void car_stop() {
  analogWrite(pwm_motor_bl, 0);
  analogWrite(pwm_motor_br, 0);
  analogWrite(pwm_motor_fl, 0);
  analogWrite(pwm_motor_fr, 0);
}

void motor_init() {
  pinMode(pwm_motor_fl, OUTPUT);
  pinMode(pwm_motor_fr, OUTPUT);
  pinMode(pwm_motor_bl, OUTPUT);
  pinMode(pwm_motor_br, OUTPUT);
  pinMode(encoder_left, INPUT);
  pinMode(encoder_right, INPUT);
  analogWrite(pwm_motor_fl, base_sp);
  analogWrite(pwm_motor_fr, base_sp);
  analogWrite(pwm_motor_bl, base_sp);
  analogWrite(pwm_motor_br, base_sp);
}

void line_detecter_init() {
  pinMode(line_left, INPUT);
  pinMode(line_center, INPUT);
  pinMode(line_right, INPUT);
}

void setup() {
  Serial.begin(9600);
  line_detecter_init();
  motor_init();
}

void loop() {
  if(my_delay(10)) {
    get_current_position();
    // Serial.print("Current pos: ");
    // Serial.println(current_pos);
    PID_control();
    motor_control();
    // Serial.print("Output: ");
    // Serial.println(delta);
    //Serial.print("L: "); Serial.print(line_left_pos);
    // Serial.print(" C: "); Serial.print(line_center_pos);
    // Serial.print(" R: "); Serial.println(line_right_pos);
  }
}
