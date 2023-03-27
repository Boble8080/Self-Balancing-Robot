#define Lmotor1 19
#define Lmotor2 18
#define LmotorEn 5

#define potPin 36

int pwmLeft = 0;
int pwmHz = 1000; // PWM frequency of 1 KHz
int pwmRes = 8; // 8-bit resolution, 256 possible values

void leftMotor(int speed);

void setup()
{
  ledcSetup(pwmLeft, pwmHz, pwmRes);
  ledcAttachPin(LmotorEn, pwmLeft);
  pinMode(Lmotor1, OUTPUT);
  pinMode(Lmotor2, OUTPUT);

  Serial.begin(115200);
  analogReadResolution(9);
}

void loop()
{
  int potval = analogRead(potPin) - 256;
  leftMotor(potval);
}

void leftMotor(int speed)
{
  //Serial.println(speed);
  if (speed == 0)
  {
    digitalWrite(Lmotor1, 0);
    digitalWrite(Lmotor2, 0);
    ledcWrite(pwmLeft, 0);
    Serial.println(speed);
  }
  else if (speed > 0)
  {
    digitalWrite(Lmotor1, 1);
    digitalWrite(Lmotor2, 0);
    ledcWrite(pwmLeft, speed);
    Serial.println(speed);
  }
  else if (speed < 0)
  {
    digitalWrite(Lmotor1, 0);
    digitalWrite(Lmotor2, 1);
    ledcWrite(pwmLeft, 256-speed);
    Serial.println(speed);
  }
}
