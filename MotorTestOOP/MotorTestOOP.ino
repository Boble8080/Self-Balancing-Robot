#include <arduino.h>
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

class Motor 
{
  public:
    byte Pin1;
    byte Pin2;
    byte PinPWM;
    byte PWMchannel;
    
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

    volatile uint64_t StartValue = 0;                 // First interrupt value
    volatile uint64_t PeriodCount;                    // period in counts 
    float             Freq;                           // frequency
    float             RPM;
    hw_timer_t * timer = NULL;  

void IRAM_ATTR handleInterrupt()
  {
    uint64_t TempVal = timerRead(timer);            // value of timer at interrupt
    PeriodCount = TempVal - StartValue;             // period count between rising edges
    StartValue = TempVal;                           // puts latest reading as start for next calculation
  }

// class Encoder 
// {
//   private:
    

//   public:
//     byte interruptPin;
    
  
  
//   float freq()
//   {
//     Freq = 40000000.00 / PeriodCount;
//     return Freq;
//   }
//   float rpm()
//   {
//     Freq = 40000000.00 / PeriodCount;// calculate frequency 
//     RPM = (Freq*6.0)/49.0;
//     return RPM;
//   }
//   Encoder(byte pin, handleInterrupt())
//   {
//     this->interruptPin = pin;
//     pinMode(interruptPin, INPUT);
//     attachInterrupt(interruptPin, handleInterrupt, FALLING);            // attaches pin to interrupt on Falling Edge
//     timer = timerBegin(0, 2, true);                                     // configure timer 
//     // 0 = first timer
//     // 2 is prescaler so 80 MHZ divided by 2 = 40 MHZ signal
//     // true - counts up
//     timerStart(timer);
//   }
// };

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
