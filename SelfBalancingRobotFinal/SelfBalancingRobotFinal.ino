#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
//#include <PID_v1.h>
#include "math.h"

#define MpuInterruptPin 23

// Left motor pins
#define Lmotor1 18
#define Lmotor2 19
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

#define pwmHz 1350  // PWM frequency in Hz
#define pwmRes 8    // PWM resolution

/////////////////////////////////////////////////////////////////////////
//                              PID                                    //
/////////////////////////////////////////////////////////////////////////
double Setpoint = 0;
double Input, Output;
double Kp = 7.6, Ki = 0.1180, Kd = 0.15;

class PID {
public:

  double kp;  // * (P)roportional Tuning Parameter
  double ki;  // * (I)ntegral Tuning Parameter
  double kd;  // * (D)erivative Tuning Parameter

  double* inputPointer;     // * Pointers to the Input, Output, and Setpoint variables
  double* outputPointer;    //   This creates a hard link between the variables and the PID
  double* setpointPointer; 

  uint32_t lastTime;
  double integralSum, lastInput;

  uint32_t SampleTime = 4;
  double outMin = -255;
  double outMax = 255;
  bool Enabled;

  double deltaTime = (double)SampleTime;


  PID(double* Input, double* Output, double* Setpoint, double inKp, double inKi, double inKd) {
    outputPointer = Output;
    inputPointer = Input;
    setpointPointer = Setpoint;
    Enabled = false;
    kp = inKp;
    ki = inKi;
    kd = inKd;
    lastTime = millis() - SampleTime;
  }

  void setEnable(int Mode) {
    bool newMode = (Mode == 1);
    if (newMode && !Enabled) {
      integralSum = *outputPointer;
      lastInput = *inputPointer;
      CheckLimits(integralSum);
    }
    Enabled = Mode;
  }
  bool Compute() {
    if (!Enabled) return false;
    uint32_t timeChange = (millis() - lastTime);
    // Only triggers pid if the sample time has passed
    if (timeChange >= SampleTime) {
      double input = *inputPointer;
      double error = *setpointPointer - input;
      double dInput = (input - lastInput);
      double output;

      // P term
      output = kp * error;

      // I term
      integralSum += (ki * deltaTime * error);
      output += CheckLimits(integralSum);

      // D term
      output -= (kd / deltaTime) * dInput;
      output = CheckLimits(output);
      *outputPointer = output;

      // Save values for next calculation
      lastInput = input;
      lastTime = millis();
      return true;
    } else return false;
  }
  double CheckLimits(double input) {

    if (input > outMax) {
      input = outMax;
    } else if (input < outMin) {
      input = outMin;
    }
    return input;
  }
};
PID balancePID(&Input, &Output, &Setpoint, Kp, Ki, Kd);

double C_Setpoint = 0;
double C_Input;
double C_Output;
double C_Kp = 0.01, C_Ki = 0.0001, C_Kd = 0.0001;

PID CascadePID(&C_Input, &C_Output, &C_Setpoint, C_Kp, C_Ki, C_Kd);

CascadePID.outMin = -5.0;
CascadePID.outMax = 5.0;

/////////////////////////////////////////////////////////////////////////
//                               Motor                                 //
/////////////////////////////////////////////////////////////////////////

class Motor {
public:
  int8_t speedOffset = 0;
  uint8_t Pin1;
  uint8_t Pin2;
  uint8_t PinPWM;
  uint8_t PWMchannel;
  Motor(uint8_t inPin1, uint8_t inPin2, uint8_t inPinPWM, uint8_t inPWMchannel) {
    Pin1 = inPin1;
    Pin2 = inPin2;
    PinPWM = inPinPWM;
    PWMchannel = inPWMchannel;
    ledcSetup(PWMchannel, pwmHz, pwmRes);
    ledcAttachPin(PinPWM, PWMchannel);
    pinMode(Pin1, OUTPUT);
    pinMode(Pin2, OUTPUT);
  }
  void rotate(int16_t speed) {
    if (speed == 0) {
      digitalWrite(Pin1, 0);
      digitalWrite(Pin2, 0);
      ledcWrite(PWMchannel, 0);
    } else if (speed > 0) {
      digitalWrite(Pin1, 1);
      digitalWrite(Pin2, 0);
      ledcWrite(PWMchannel, speed);
    } else if (speed < 0) {
      digitalWrite(Pin1, 0);
      digitalWrite(Pin2, 1);
      ledcWrite(PWMchannel, abs(speed));
    }
  }
};

Motor leftMotor(Lmotor1, Lmotor2, LmotorEn, LpwmChannel);
Motor rightMotor(Rmotor1, Rmotor2, RmotorEn, RpwmChannel);

float measureCurrent() {
  float reading = analogRead(currentMeter);
  Serial.print(" ");
  Serial.print(reading - 230.0);
  Serial.print("\t ");
  return reading;
}

void motorEnable(bool enable) {
  digitalWrite(MotorENABLE, enable);
}

/////////////////////////////////////////////////////////////////////////
//                          Encoder                                    //
/////////////////////////////////////////////////////////////////////////
float distance = 0;

const byte LeftEncoderInterrupt = Lencoder;  // Assign the interrupt pin
volatile uint64_t LeftStartValue = 0;        // First interrupt value
volatile uint64_t LeftPeriodCount;           // period in counts
float LeftFreq;                              // frequency
float LeftRPM;
volatile int64_t LeftSteps = 0;
hw_timer_t* LeftTimer = NULL;  // pointer to a variable of type hw_timer_t


const byte RightEncoderInterrupt = Rencoder;  // Assign the interrupt pin
volatile uint64_t RightStartValue = 0;        // First interrupt value
volatile uint64_t RightPeriodCount;           // period in counts
float RightFreq;                              // frequency
float RightRPM;
volatile int64_t RightSteps = 0;
hw_timer_t* RightTimer = NULL;  // pointer to a variable of type hw_timer_t

void IRAM_ATTR handleLeftInterrupt() {
  uint64_t TempVal = timerRead(LeftTimer);     // value of timer at interrupt
  LeftPeriodCount = TempVal - LeftStartValue;  // period count between rising edges
  LeftStartValue = TempVal;                    // puts latest reading as start for next calculation
  if (Output > 0) LeftSteps++;
  else LeftSteps--;
}

void IRAM_ATTR handleRightInterrupt() {
  uint64_t TempVal = timerRead(RightTimer);      // value of timer at interrupt
  RightPeriodCount = TempVal - RightStartValue;  // period count between rising edges
  RightStartValue = TempVal;                     // puts latest reading as start for next calculation
  if (Output > 0)
  {
     RightSteps++;
     distance += 1.659;
  }
  else 
  {
     RightSteps--;
     distance -= 1.659;
  }
}

float LeftFrequency() {
  LeftFreq = 40000000.00 / LeftPeriodCount;
  return LeftFreq;
}

float RightFrequency() {
  RightFreq = 40000000.00 / RightPeriodCount;
  return RightFreq;
}

float returnLeftRPM() {
  LeftRPM = (LeftFrequency() * 6.0) / 49.0;
  return LeftRPM;
}

float returnRightRPM() {
  RightRPM = (RightFrequency() * 6.0) / 49.0;
  return RightRPM;
}

uint16_t linearCalCutoff = 500;
uint16_t angleCalCutoff = 50;
float linearOffset = 0;
uint64_t linearCalTimer = 0;
uint16_t linearCalInterval = 3000;

float linearCalibration() {
  // Linear Calibration
  if (linearOffset >= 5.0) {
    return linearOffset;
  } else if (millis() - linearCalTimer < linearCalInterval) {
    return linearOffset;
  }
  // Left step count is bigger than the right
  else if (abs(LeftSteps) >= abs(RightSteps)) {
    // and is above cuttoff
    if (abs(LeftSteps) < linearCalCutoff) {
      return linearOffset;
    }
    // and is positive
    if (LeftSteps > 0) {
      // increase offset and reset Steps
      linearOffset += 0.05;
      LeftSteps -= RightSteps;
      RightSteps = 0;
    }
    // or is negative
    else {
      // decrease offset and reset Steps
      linearOffset -= 0.05;
      LeftSteps += RightSteps;
      RightSteps = 0;
    }
  }
  // Right step count is bigger than the Left
  else {
    // and is above cuttoff
    if (abs(RightSteps) < linearCalCutoff) {
      return linearOffset;
    }
    // and is positive
    if (RightSteps > 0) {
      // increase offset and reset Steps
      linearOffset += 0.1;
      RightSteps -= LeftSteps;
      LeftSteps = 0;
    } else {
      // increase offset and reset Steps
      linearOffset -= 0.1;
      RightSteps += LeftSteps;
      LeftSteps = 0;
    }
  }
  linearCalTimer = millis();
  return linearOffset;
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
float remoteSetpoint = 0;
void printSerialDebug() {
  Serial.print(" ");
  Serial.print(Input);
  Serial.print("\t");
  Serial.print(Setpoint);
  Serial.print("\t");
  Serial.print(Output);
  Serial.print("\t");
  Serial.print(linearOffset);
  Serial.print("\t");
  Serial.print(distance);
  Serial.print("\t");
  Serial.print(RightSteps);
  Serial.print("\t");
  Serial.print(returnRightRPM());
  Serial.print("\t");
  Serial.print(remoteSetpoint);
  Serial.println();
}

void debugGyro() {
  Serial.print("Gyro:ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
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

    Input = -ypr[1] * 180 / M_PI;
    if (Input <= 0.1 && Input >= -0.1) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else digitalWrite(LED_BUILTIN, LOW);
  }
}

bool fallFlag = false;

bool testForFall() {
  // Robot is currently fallen
  if (fallFlag == true) {
    motorEnable(false);
    balancePID.setEnable(0);
    digitalWrite(LED_BUILTIN, HIGH);
    // Robot has been lifted up
    if (abs(Input) <= 10) {
      fallFlag = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
    return 1;
  }
  // Robot has not fallen
  else {
    motorEnable(true);
    balancePID.setEnable(1);
    // Robot just fell
    if (abs(Input) >= 50.0) {
      fallFlag = true;
    }
    return 0;
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

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

#define BLUETOOTH_ENABLED_0



int8_t remoteCtrlLeft = 0;
int8_t remoteCtrlRight = 0;

#ifdef BLUETOOTH_ENABLED
// RemoteXY select connection mode and include library
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>

#include <RemoteXY.h>

// RemoteXY connection settings
#define REMOTEXY_BLUETOOTH_NAME "SelfBalancingRobot"


// RemoteXY configurate
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 85 bytes
  { 255, 2, 0, 204, 0, 78, 0, 16, 170, 2, 5, 32, 61, 13, 50, 50, 2, 46, 58, 58,
    2, 26, 31, 71, 56, 244, 21, 61, 61, 4, 13, 54, 54, 0, 2, 24, 75, 0, 0, 180,
    194, 0, 0, 180, 66, 0, 0, 240, 65, 0, 0, 32, 65, 0, 0, 160, 64, 24, 0, 67,
    4, 0, 0, 100, 6, 0, 7, 63, 7, 2, 26, 100, 67, 4, 0, 7, 100, 6, 0, 255,
    63, 7, 2, 26, 100 };

// this structure defines all the variables and events of your control interface
struct {

  // input variables
  int8_t joystick_x;  // from -100 to 100
  int8_t joystick_y;  // from -100 to 100

  // output variables
  float Angle;         // from -90 to 90
  char textBox[100];   // string UTF8 end zero
  char textBox2[100];  // string UTF8 end zero

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)



void remoteControl() {

  RemoteXY.Angle = (float)Input;

  remoteSetpoint = (float)RemoteXY.joystick_y * -0.1;

  remoteCtrlLeft = (-RemoteXY.joystick_x * 0.5);
  remoteCtrlRight = (RemoteXY.joystick_x * 0.5);

  // If the remote control is being used, dont calibrate setpoint
  if (abs(remoteSetpoint) > 0) {
    RightSteps = 0;
    LeftSteps = 0;
  }
}

int8_t rotationDegrees = 0;
int8_t targetAngle = 0;
void turnToAngle(int8_t rotationDegreesInput) {
  rotationDegrees += rotationDegreesInput;
  targetAngle = (Input) + rotationDegrees;
}


void formatText() {
  char outputString[100];
  char stringBuffer[16];
  strcpy(outputString, "Dist: ");
  dtostrf(distance, 3, 2, stringBuffer);
  strcat(outputString, stringBuffer);

  strcat(outputString, " \tlinCal: ");
  dtostrf(linearCalibration(), 3, 2, stringBuffer);
  strcat(outputString, stringBuffer);

  strcpy(RemoteXY.textBox, outputString);

  // Second String
  strcpy(outputString, "Setpoint: ");
  dtostrf(Setpoint, 3, 2, stringBuffer);
  strcat(outputString, stringBuffer);

  strcat(outputString, " \tRPM: ");
  dtostrf(returnLeftRPM(), 3, 2, stringBuffer);
  strcat(outputString, stringBuffer);

  strcpy(RemoteXY.textBox2, outputString);
}

#endif  //BLUETOOTH_ENABLED
/////////////////////////////////////////////////////////////////////////
//                          Setup & Loop                               //
/////////////////////////////////////////////////////////////////////////

void setup() {
#ifdef BLUETOOTH_ENABLED
  RemoteXY_Init();
#endif  //BLUETOOTH_ENABLED
  Wire.begin();
  Wire.setClock(400000);  
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

  // Configures encoder interrupts
  pinMode(Lencoder, INPUT);
  attachInterrupt(Lencoder, handleLeftInterrupt, RISING);
  LeftTimer = timerBegin(0, 2, true);
  timerStart(LeftTimer);

  pinMode(Rencoder, INPUT);
  attachInterrupt(Rencoder, handleRightInterrupt, RISING);
  RightTimer = timerBegin(1, 2, true);
  timerStart(LeftTimer);

  analogReadResolution(9);
  pinMode(MotorENABLE, OUTPUT);

  balancePID.setEnable(1);
  mpu.setDLPFMode(3);

  readMPU();
  delay(2000);
  readMPU();
  while (Input >= 10 && Input <= 10) {
    readMPU();
    motorEnable(0);
    LEDflash(300);
  }
  motorEnable(1);
}

void loop() {
#ifdef BLUETOOTH_ENABLED
  RemoteXY_Handler();    // Updates Bluetooth
  remoteControl();       // Applies Bluetooth control to setpoint 
  formatText();          // Sends text to Bluetooth 
  // Flashes the LED if Bluetooth is not connected
  if(!RemoteXY.connect_flag) LEDflash(500);
  else LEDflash(0);
#endif  //BLUETOOTH_ENABLED

  readMPU(); // Reads Gyroscope
  
  // Adjust Setpoint
  Setpoint = remoteSetpoint + C_Output;
  CascadePID.Compute();
  
  // Updates the PID and writes debug data to the serial monitor 
  //while the PID function is inactive.  
  if (!balancePID.Compute()) printSerialDebug(); 
  testForFall();// Checks if robot fell over, if so stops the motors
  
  // Adjust Motor Speed
  leftMotor.rotate(Output + remoteCtrlLeft);
  rightMotor.rotate(Output + remoteCtrlRight);
}
