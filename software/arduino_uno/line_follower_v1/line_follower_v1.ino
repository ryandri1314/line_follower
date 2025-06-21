#define line_left A0
#define line_center A1
#define line_right A2
#define encoder_right 13
#define encoder_left 7
#define pwm_motor_fl 3
#define pwm_motor_fr 5
#define pwm_motor_bl 6
#define pwm_motor_br 9 

uint8_t log_line_1, log_line_2, log_line_3;

void setup() {
  // put your setup code here, to run once:
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
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  log_line_1 = analogRead(line_1);
  log_line_2 = analogRead(line_2);
  log_line_3 = analogRead(line_3);
  Serial.print("log_line_1: ");
  Serial.println(log_line_1);
  Serial.print("log_line_2: ");
  Serial.println(log_line_2);
  Serial.print("log_line_3: ");
  Serial.println(log_line_3);
}
