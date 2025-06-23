#define line_left A0
#define line_center A1
#define line_right A2
#define encoder_right 12
#define encoder_left 7
#define pwm_motor_fl 5
#define pwm_motor_fr 3
#define pwm_motor_bl 9
#define pwm_motor_br 6 

const float Kp = 0.25;
const float Ki = 0.05;
const float Kd = 0.0001;
const float desired_pos = 0.0;
const float dt = 0.01;
uint8_t line_left_pos, line_right_pos, line_center_pos;
float current_pos, prev_pos, error, derivative, output, prev_error;
float integral = 0;
unsigned long encoder_left_count = 0, encoder_right_count = 0;
unsigned long last_time = 0;
uint8_t sp_fl = 128, sp_fr = 128, sp_bl = 128, sp_br = 128;
unsigned long last_time_reset;
uint8_t time_reset = 100000;

bool my_delay(uint8_t time) {
  if (millis() - last_time > time) {
    last_time = millis();
    return true;
  }
  return false;
}

void PID_control() {
  error = desired_pos - current_pos;
  if (millis() - last_time_reset > time_reset) {
    last_time_reset = millis();
    integral = 0;
  } else {
    integral += (error * dt);
  }
  derivative = (error - prev_error) / dt;
  output = Kp*error + Ki*integral + Kd*derivative;
  prev_error = error;
}

void motor_control() {
  if (output > 0) {
    sp_fr += output;
    if (sp_fr > 255) {
      sp_fr = 255;
    }
    sp_br += output;
    if (sp_br > 255) {
      sp_br = 255;
    }
    sp_fl -= output;
    if (sp_fl < 0) {
      sp_fl = 0;
    }
    sp_bl -= output;
    if (sp_bl < 0) {
      sp_bl = 0;
    }
  } else {
    sp_fl += output;
    if (sp_fl > 255) {
      sp_fl = 255;
    }
    sp_bl += output;
    if (sp_bl > 255) {
      sp_bl = 255;
    }
    sp_fr -= output;
    if (sp_fr < 0) {
      sp_fr = 0;
    }
    sp_br -= output;
    if (sp_br < 0) {
      sp_br = 0;
    }
  }
}

void get_current_position() {
  line_left_pos = analogRead(line_left);
  line_center_pos = analogRead(line_center);
  line_right_pos = analogRead(line_right);

  uint8_t sum = line_left_pos + line_center_pos + line_right_pos;

  if (sum > 0) {
    current_pos = ((line_left_pos * (-1)) + (line_center_pos * 0) + (line_right_pos * 1))/(float)sum;
    prev_pos = current_pos;
  } else {
    //Control exception
    current_pos = prev_pos;
  }
}

void car_stop() {
  digitalWrite(pwm_motor_bl, LOW);
  digitalWrite(pwm_motor_br, LOW);
  digitalWrite(pwm_motor_fl, LOW);
  digitalWrite(pwm_motor_fr, LOW);
}

void motor_set() {
  analogWrite(pwm_motor_br, sp_br);
  analogWrite(pwm_motor_fl, sp_fl);
  analogWrite(pwm_motor_bl, sp_bl);
  analogWrite(pwm_motor_fr, sp_fr);
}

void motor_init() {
  pinMode(pwm_motor_fl, OUTPUT);
  pinMode(pwm_motor_fr, OUTPUT);
  pinMode(pwm_motor_bl, OUTPUT);
  pinMode(pwm_motor_br, OUTPUT);
  pinMode(encoder_left, INPUT);
  pinMode(encoder_right, INPUT);
  car_stop();
  motor_set();
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
  if(my_delay(dt*5000)) {
    get_current_position();
    Serial.print("Current pos: ");
    Serial.println(current_pos);
    PID_control();
    motor_control();
    motor_set();
    Serial.print("Output: ");
    Serial.println(output);
    // Serial.print("LEFT: ");
    // Serial.println(line_left_pos);
    // Serial.print("CENTER: ");
    // Serial.println(line_center_pos);
    // Serial.print("RIGHT: ");
    // Serial.println(line_right_pos);
  }
}
