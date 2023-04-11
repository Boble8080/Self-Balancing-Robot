#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define MpuInterruptPin 23

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

/////////////////////////////////////////////////////////////////////////
//                          Motor Class                                //
/////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////
//                          Encoder                                    //
/////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////
//                          Gyroscope                                  //
/////////////////////////////////////////////////////////////////////////

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void debugGyro() {
  Serial.print(-90);
  Serial.print(" ");
  Serial.print(90);
  Serial.print(" ");
  Serial.print(0);
  Serial.print("\t");
  Serial.print("Gyro:ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.println(ypr[2] * 180 / M_PI);
}

void debugAccel() {
  Serial.print(-1800);
  Serial.print(" ");
  Serial.print(1800);
  Serial.print(" ");
  Serial.print("aworld\t");
  Serial.print(aaWorld.x);
  Serial.print("\t");
  Serial.print(aaWorld.y);
  Serial.print("\t");
  Serial.println(aaWorld.z);
}

void debugBoth() {
  Serial.print("Acel: X: ");
  Serial.print(aaWorld.x);
  Serial.print("\t\tY: ");
  Serial.print(aaWorld.y);
  Serial.print("\t\tZ: ");
  Serial.print(aaWorld.z);
  Serial.print("\t\tGyro: Yaw: ");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\tPch: ");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\tRll: ");
  Serial.println(ypr[2] * 180 / M_PI);
}

/////////////////////////////////////////////////////////////////////////
//                          Setup & Loop                               //
/////////////////////////////////////////////////////////////////////////

void setup()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
  mpu.initialize();
  pinMode(MpuInterruptPin, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(127);
  mpu.setYGyroOffset(-101);
  mpu.setZGyroOffset(15);
  mpu.setXAccelOffset(-409); 
  mpu.setYAccelOffset(409); 
  mpu.setZAccelOffset(1331); 

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
  leftMotor.rotate(val);
  delay(1000);
  rightMotor.rotate(-val);
  leftMotor.rotate(-val);
  delay(1000);
}
