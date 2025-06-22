#define line_left A0
#define line_center A1
#define line_right A2
#define encoder_right 12
#define encoder_left 7
#define pwm_motor_fl 5
#define pwm_motor_fr 3
#define pwm_motor_bl 9
#define pwm_motor_br 6 

uint8_t log_line_1, log_line_2, log_line_3;
unsigned long encoder_left_count = 0, encoder_right_count = 0;

void setup() {
  Serial.begin(9600);
  pinMode(line_left, INPUT);
  pinMode(line_center, INPUT);
  pinMode(line_right, INPUT);
  pinMode(encoder_left, INPUT);
  pinMode(encoder_right, INPUT);
  pinMode(pwm_motor_fl, OUTPUT);
  pinMode(pwm_motor_fr, OUTPUT);
  pinMode(pwm_motor_bl, OUTPUT);
  pinMode(pwm_motor_br, OUTPUT);

  delay(1000);

  digitalWrite(pwm_motor_bl, LOW);
  digitalWrite(pwm_motor_br, LOW);
  digitalWrite(pwm_motor_fl, LOW);
  digitalWrite(pwm_motor_fr, LOW);
  
  delay(1000);

  analogWrite(pwm_motor_br, 255);
  analogWrite(pwm_motor_fl, 50);
  analogWrite(pwm_motor_bl, 50);
  analogWrite(pwm_motor_fr, 255);
}

void loop() {
  // if (digitalRead(encoder_left)) {
  //   encoder_left_count++;
  //   Serial.print("LEFT: ");
  //   Serial.println(encoder_left_count);
  // }
  
  // if (digitalRead(encoder_right)) {
  //   encoder_right_count++;
  //   Serial.print("RIGHT: ");
  //   Serial.println(encoder_right_count);
  // }
  // analogWrite(pwm_motor_bl, 100);
  // analogWrite(pwm_motor_br, 100);
  // analogWrite(pwm_motor_fl, 100);
  // analogWrite(pwm_motor_fl, 100);
  // log_line_1 = analogRead(line_1);
  // log_line_2 = analogRead(line_2);
  // log_line_3 = analogRead(line_3);
  // Serial.print("log_line_1: ");
  // Serial.println(log_line_1);
  // Serial.print("log_line_2: ");
  // Serial.println(log_line_2);
  // Serial.print("log_line_3: ");
  // Serial.println(log_line_3);
  delay(100);
}
