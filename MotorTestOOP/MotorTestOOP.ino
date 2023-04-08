// Left motor pins
#define Lmotor1 19
#define Lmotor2 18
#define LmotorEn 5
#define LpwmChannel 0

// Right motor pins
#define Rmotor1 16
#define Rmotor2 17
#define RmotorEn 4
#define RpwmChannel 1

#define potPin 36

#define pwmHz 1000 // PWM frequency of 1 KHz
#define pwmRes 8 // 8-bit resolution
byte test = 0;

class Motor {
  private:
    byte Pin1;
    byte Pin2;
    byte PinPWM;
    byte PWMchannel;
  public:
    Motor(byte Pin1, byte Pin2,byte pinPWM, byte PWMchannel)
    {
        // Stores constructor input as private variables
        this->Pin1 = Pin1;
        this->Pin2 = Pin2;
        this->PinPWM = PinPWM;
        this->PWMchannel = PWMchannel;
        ledcSetup(PWMchannel, pwmHz, pwmRes);
        ledcAttachPin(pinPWM, PWMchannel);
        pinMode(Pin1, OUTPUT);
        pinMode(Pin2, OUTPUT);
    }
    void rotate(int speed)
    {
      Serial.println(speed);
      if (speed == 0)
      {
        digitalWrite(Pin1, 0);
        digitalWrite(Pin2, 0);
        ledcWrite(PWMchannel, 0);
        Serial.println("stop");
      }
      else if (speed > 0)
      {
        digitalWrite(Pin1, 1);
        digitalWrite(Pin2, 0);
        ledcWrite(PWMchannel, speed);
        Serial.println("forward");
      }
      else if (speed < 0)
      {
        digitalWrite(Pin1, 0);
        digitalWrite(Pin2, 1);
        ledcWrite(PWMchannel, 257 - speed);
        Serial.println("back");
      }
    }
    
};

Motor leftMotor(Lmotor1, Lmotor2, LmotorEn, LpwmChannel);
Motor rightMotor(Rmotor1, Rmotor2, RmotorEn, RpwmChannel);


void setup()
{
  Serial.begin(115200);
  analogReadResolution(9);
}

int val = 0;

void loop()
{
  val = analogRead(potPin) / 2;
  rightMotor.rotate(val);
  leftMotor.rotate(val);
  delay(1000);
  rightMotor.rotate(-val);
  leftMotor.rotate(-val);
  delay(1000);
}
