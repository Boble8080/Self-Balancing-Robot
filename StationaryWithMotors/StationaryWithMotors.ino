#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <PID_v1.h>
#include "math.h"

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
#define currentMeter 35
#define MotorENABLE 27

#define pwmHz 1000  // PWM frequency of 1 KHz // test others
#define pwmRes 8    // 8-bit resolution

double Setpoint = 0;
double Input, Output;
double Kp = 7.6, Ki = 118.00, Kd = 0.15;
PID balancePID(&Input, &Output, &Setpoint, Kp, Ki, Kd,DIRECT);


/////////////////////////////////////////////////////////////////////////
//                               Motor                                 //
/////////////////////////////////////////////////////////////////////////

class Motor {
  public:
    byte Pin1;
    byte Pin2;
    byte PinPWM;
    byte PWMchannel;
    Motor(byte inPin1, byte inPin2, byte inPinPWM, byte inPWMchannel) {
      // Stores constructor input as private variables
      Pin1 = inPin1;
      Pin2 = inPin2;
      PinPWM = inPinPWM;
      PWMchannel = inPWMchannel;
      ledcSetup(PWMchannel, pwmHz, pwmRes);
      ledcAttachPin(PinPWM, PWMchannel);
      pinMode(Pin1, OUTPUT);
      pinMode(Pin2, OUTPUT);
    }
    void rotate(int speed) {
      //motorGain = 20;
      //Serial.println(speed);
      if (speed == 0) {
        digitalWrite(Pin1, 0);
        digitalWrite(Pin2, 0);
        ledcWrite(PWMchannel, 0);
        //Serial.print("\t");
        //Serial.print(abs(speed));
      } else if (speed > 0) {
        digitalWrite(Pin1, 1);
        digitalWrite(Pin2, 0);
        ledcWrite(PWMchannel, speed );
        //Serial.print("\t");
        //Serial.print(speed + motorGain);
      } else if (speed < 0) {
        digitalWrite(Pin1, 0);
        digitalWrite(Pin2, 1);
        ledcWrite(PWMchannel, abs(speed));
        //Serial.print("\t");
        //Serial.print(abs(speed) + motorGain);
      }
    }
};

Motor leftMotor(Lmotor1, Lmotor2, LmotorEn, LpwmChannel);
Motor rightMotor(Rmotor1, Rmotor2, RmotorEn, RpwmChannel);

float measureCurrent()
{
  float reading = analogRead(currentMeter);
  Serial.print("\t");
  Serial.print(reading - 230.0);
  Serial.print(" ");
  return reading;

}


void motorEnable(bool enable)
{
  digitalWrite(MotorENABLE, enable);
}


/////////////////////////////////////////////////////////////////////////
//                          Encoder                                    //
/////////////////////////////////////////////////////////////////////////

const byte LeftEncoderInterrupt = 36;  // Assign the interrupt pin
volatile uint64_t LeftStartValue = 0;  // First interrupt value
volatile uint64_t LeftPeriodCount;     // period in counts
float LeftFreq;                        // frequency
float LeftRPM;
hw_timer_t* LeftTimer = NULL;  // pointer to a variable of type hw_timer_t


const byte RightEncoderInterrupt = 39;  // Assign the interrupt pin
volatile uint64_t RightStartValue = 0;  // First interrupt value
volatile uint64_t RightPeriodCount;     // period in counts
float RightFreq;                        // frequency
float RightRPM;
hw_timer_t* RightTimer = NULL;  // pointer to a variable of type hw_timer_t

void IRAM_ATTR handleLeftInterrupt() {
  uint64_t TempVal = timerRead(LeftTimer);     // value of timer at interrupt
  LeftPeriodCount = TempVal - LeftStartValue;  // period count between rising edges
  LeftStartValue = TempVal;                    // puts latest reading as start for next calculation
}

void IRAM_ATTR handleRightInterrupt() {
  uint64_t TempVal = timerRead(RightTimer);      // value of timer at interrupt
  RightPeriodCount = TempVal - RightStartValue;  // period count between rising edges
  RightStartValue = TempVal;                     // puts latest reading as start for next calculation
}

float LeftFrequency() {
  LeftFreq = 40000000.00 / LeftPeriodCount;
  return RightFreq;
}

float RightFrequency() {
  RightFreq = 40000000.00 / RightPeriodCount;
  return RightFreq;
}

float returnLeftRPM() {
  LeftFreq = 40000000.00 / LeftPeriodCount;  // calculate frequency
  LeftRPM = (LeftFreq * 6.0) / 49.0;
  return LeftRPM;
}

float returnRightRPM() {
  RightFreq = 40000000.00 / RightPeriodCount;  // calculate frequency
  RightRPM = (RightFreq * 6.0) / 49.0;
  return RightRPM;
}



/////////////////////////////////////////////////////////////////////////
//                          Gyroscope                                  //
/////////////////////////////////////////////////////////////////////////

MPU6050 mpu;

bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void debugAngle()
{
  Serial.print("Gyro: ");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t Accel:");
  Serial.print(aaWorld.y);
  Serial.print("\t ");
}

void debugGyro() {
  Serial.print("Gyro:ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[2] * 180 / M_PI);
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
  Serial.print(aaWorld.z);
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
  Serial.print(ypr[2] * 180 / M_PI);
}
void readMPU() {
  if (!dmpReady) {
    LEDflash(2000);
    return;
  }
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    digitalWrite(LED_BUILTIN, LOW);
    Input = -ypr[1] * 180 / M_PI;
  }
}

/////////////////////////////////////////////////////////////////////////
//                               LED                                   //
/////////////////////////////////////////////////////////////////////////

bool LEDstate = 0;
uint32_t LEDoldTime = 0;
void LEDflash(uint16_t speed)  // Led flash in ms
{
  if (speed == 0) {
    digitalWrite(LED_BUILTIN, 0);
  } else if (millis() - LEDoldTime >= speed) {
    LEDoldTime = millis();
    LEDstate = !LEDstate;
    digitalWrite(LED_BUILTIN, LEDstate);
  }
}

/////////////////////////////////////////////////////////////////////////
//                          Setup & Loop                               //
/////////////////////////////////////////////////////////////////////////


void setup() {
  
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
  mpu.initialize();
  pinMode(MpuInterruptPin, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(148);
  mpu.setYGyroOffset(-102);
  mpu.setZGyroOffset(4);
  mpu.setXAccelOffset(-539);
  mpu.setYAccelOffset(369);
  mpu.setZAccelOffset(1386);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(MpuInterruptPin));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(MpuInterruptPin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(Lencoder, INPUT);
  attachInterrupt(Lencoder, handleLeftInterrupt, FALLING);
  LeftTimer = timerBegin(0, 2, true);
  timerStart(LeftTimer);

  pinMode(Rencoder, INPUT);
  attachInterrupt(Rencoder, handleRightInterrupt, FALLING);
  RightTimer = timerBegin(1, 2, true);
  timerStart(LeftTimer);

  analogReadResolution(9);


  balancePID.SetMode(1);
  balancePID.SetOutputLimits(-256, 256);
  balancePID.SetSampleTime(100);
  motorEnable(1);
}


void loop() {
  readMPU();
  if (Input >= 50 || Input <= -50 || measureCurrent() <= -30.0)
  {
    while (1) {
      leftMotor.rotate(0);
      rightMotor.rotate(0);
      Serial.println(analogRead(potPin) / 1.0);
      digitalWrite(LED_BUILTIN, HIGH);
      measureCurrent();
      delay(100);
    }
  }
  else motorEnable(1);
  if (Input <= 0.1 && Input >= -0.1)
  {
    //balancePID.outputSum = balancePID.outputSum/2;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  balancePID.Compute();
  debugAngle();
  Serial.print("\t");
  Serial.print(Output);
  leftMotor.rotate(Output);
  rightMotor.rotate(Output);
  measureCurrent();
  Serial.println();
  digitalWrite(LED_BUILTIN, 0);
}
