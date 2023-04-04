#define Lmotor1 19
#define Lmotor2 18
#define LmotorEn 5

#define Rmotor1 16
#define Rmotor2 17
#define RmotorEn 4


#define potPin 36

int pwmLeft = 0;
int pwmRight = 1;
int pwmHz = 1000; // PWM frequency of 1 KHz
int pwmRes = 8; // 8-bit resolution, 256 possible values

void leftMotor(int speed);
void rightMotor(int speed);

void setup()
{
  ledcSetup(pwmLeft, pwmHz, pwmRes);
  ledcSetup(pwmRight, pwmHz, pwmRes);
  ledcAttachPin(LmotorEn, pwmLeft);
  ledcAttachPin(RmotorEn, pwmRight);
  pinMode(Lmotor1, OUTPUT);
  pinMode(Lmotor2, OUTPUT);
  pinMode(Rmotor1, OUTPUT);
  pinMode(Rmotor2, OUTPUT);

  Serial.begin(115200);
  analogReadResolution(9);
}

int val = 0;

void loop()
{
  val = analogRead(potPin)/2;
  Serial.println(val);
  rightMotor(val);
  leftMotor(val);
  Serial.println("forward");
  delay(1000);
  rightMotor(-val);
  leftMotor(-val);
  Serial.println("backwards");
  delay(1000);
}

void leftMotor(int speed)
{
  if (speed == 0)
  {
    digitalWrite(Lmotor1, 0);
    digitalWrite(Lmotor2, 0);
    ledcWrite(pwmLeft, 0);
  }
  else if (speed > 0)
  {
    digitalWrite(Lmotor1, 1);
    digitalWrite(Lmotor2, 0);
    ledcWrite(pwmLeft, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(Lmotor1, 0);
    digitalWrite(Lmotor2, 1);
    ledcWrite(pwmLeft, 257-speed);
  }
}

void rightMotor(int speed)
{
  if (speed == 0)
  {
    digitalWrite(Rmotor1, 0);
    digitalWrite(Rmotor2, 0);
    ledcWrite(pwmRight, 0);
  }
  else if (speed > 0)
  {
    digitalWrite(Rmotor1, 1);
    digitalWrite(Rmotor2, 0);
    ledcWrite(pwmRight, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(Rmotor1, 0);
    digitalWrite(Rmotor2, 1);
    ledcWrite(pwmRight, 257-speed);
  }
}
