//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP32CORE_BLUETOOTH
#include <BluetoothSerial.h>
#include <RemoteXY.h>
#define REMOTEXY_BLUETOOTH_NAME "SBR" 
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =   // 72 bytes
  { 255,2,0,12,0,65,0,16,29,2,5,32,62,16,30,30,14,61,34,34,
  2,26,31,71,56,6,16,33,33,13,17,35,35,8,2,24,75,0,0,180,
  194,0,0,180,66,0,0,52,66,0,0,112,65,0,0,160,64,24,0,67,
  4,42,10,20,5,23,54,20,5,2,26,11 };
struct {

    // input variables
  int8_t Control_x; // from -100 to 100  
  int8_t Control_y; // from -100 to 100  

    // output variables
  int8_t Dial;  // from -90 to 90 
  char Text[11];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////



void setup() 
{
  RemoteXY_Init ();
  Serial.begin(115200);
}

void loop() 
{ 
  RemoteXY_Handler ();
  RemoteXY.Dial = 10;


}
