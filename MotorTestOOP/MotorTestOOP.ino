//#include <arduino.h>
// Left motor pins
#define Lmotor1 19
#define Lmotor2 18
#define LmotorEn 5
#define LpwmChannel 0
#define Lencoder 36

// Right motor pins
#define Rmotor1 16
#define Rmotor2 17
#define RmotorEn 4
#define RpwmChannel 1
#define Rencoder 39

#define potPin 34

#define pwmHz 1000 // PWM frequency of 1 KHz // test others
#define pwmRes 8 // 8-bit resolution
byte test = 0;

class Motor
{
  public:
    byte Pin1;
    byte Pin2;
    byte PinPWM;
    byte PWMchannel;

    Motor(byte Pin1, byte Pin2, byte pinPWM, byte PWMchannel)
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


const byte        LeftEncoderInterrupt = 36;          // Assign the interrupt pin
volatile uint64_t LeftStartValue = 0;                 // First interrupt value
volatile uint64_t LeftPeriodCount;                    // period in counts
float             LeftFreq;                           // frequency
float             LeftRPM;
hw_timer_t *      LeftTimer = NULL;                        // pointer to a variable of type hw_timer_t


const byte        RightEncoderInterrupt = 39;          // Assign the interrupt pin
volatile uint64_t RightStartValue = 0;                 // First interrupt value
volatile uint64_t RightPeriodCount;                    // period in counts
float             RightFreq;                           // frequency
float             RightRPM;
hw_timer_t *      RightTimer = NULL;                        // pointer to a variable of type hw_timer_t

void IRAM_ATTR handleLeftInterrupt()
{
  uint64_t TempVal = timerRead(LeftTimer);            // value of timer at interrupt
  LeftPeriodCount = TempVal - LeftStartValue;             // period count between rising edges
  LeftStartValue = TempVal;                           // puts latest reading as start for next calculation
}

void IRAM_ATTR handleRightInterrupt()
{
  uint64_t TempVal = timerRead(RightTimer);            // value of timer at interrupt
  RightPeriodCount = TempVal - RightStartValue;             // period count between rising edges
  RightStartValue = TempVal;                           // puts latest reading as start for next calculation
}

float LeftFrequency()
{
  LeftFreq = 40000000.00 / LeftPeriodCount;
  return RightFreq;
}

float RightFrequency()
{
  RightFreq = 40000000.00 / RightPeriodCount;
  return RightFreq;
}

float returnLeftRPM()
{
  LeftFreq = 40000000.00 / LeftPeriodCount;// calculate frequency
  LeftRPM = (LeftFreq * 6.0) / 49.0;
  return LeftRPM;
}

float returnRightRPM()
{
  RightFreq = 40000000.00 / RightPeriodCount;// calculate frequency
  RightRPM = (RightFreq * 6.0) / 49.0;
  return RightRPM;
}




Motor leftMotor(Lmotor1, Lmotor2, LmotorEn, LpwmChannel);
Motor rightMotor(Rmotor1, Rmotor2, RmotorEn, RpwmChannel);


void setup()
{
  Serial.begin(115200);

  pinMode(Lencoder, INPUT);
  attachInterrupt(Lencoder, handleLeftInterrupt, FALLING);
  LeftTimer = timerBegin(0, 2, true);
  timerStart(LeftTimer);

  pinMode(Rencoder, INPUT);
  attachInterrupt(Rencoder, handleRightInterrupt, FALLING);
  RightTimer = timerBegin(1, 2, true); 
  timerStart(LeftTimer);

  analogReadResolution(9);
}

int val = 0;

void loop()
{
  val = analogRead(potPin) / 2;
  rightMotor.rotate(val);
  leftMotor.rotate(0);
  
}
