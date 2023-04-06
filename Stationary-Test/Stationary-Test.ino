#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

const byte MpuInterruptPin = 23;

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

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(MpuInterruptPin, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(127);
  mpu.setYGyroOffset(-101);
  mpu.setZGyroOffset(15);
  mpu.setXAccelOffset(-409); 
  mpu.setYAccelOffset(409); 
  mpu.setZAccelOffset(1331); 



  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
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
}

void loop() {
  //while(1){}
  // If programming failed, don't try to do anything
  if (!dmpReady) {
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  }
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    debugBoth();
    digitalWrite(LED_BUILTIN, LOW);
  }
}
