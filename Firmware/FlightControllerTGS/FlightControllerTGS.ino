
#include <Wire.h>
#include <FlightControllerTGS.h>

FlightControllerTGSInput ReceiverInput(RISING);

float
  Pitch,
  Roll,
  Yaw;

float
  CalibrationPitch,
  CalibrationRoll,
  CalibrationYaw;

int CalibrationNumber;
float ReceiverValue[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int ChannelNumber = 0;

float
  Voltage,
  Current,
  BatteryLevel,
  BatteryStart;

float CurrentConsumed = 0;
float BatteryDefault = 1300;  // Altere de acordo com a mah da sua bateria
uint32_t LoopTimer;

float
  RollDesired,
  PitchDesired,
  YawDesired;

float
  ErrorRoll,
  ErrorPitch,
  ErrorYaw;

float
  InputRoll,
  InputThrottle,
  InputPitch,
  InputYaw;

float
  PrevErrorRoll,
  PrevErrorPitch,
  PrevErrorYaw;

float
  PrevItermRoll,
  PrevItermPitch,
  PrevItermYaw;

float PIDReturn[] = { 0, 0, 0 };

float PRoll = 0.6;
float IRoll = 3.5;
float DRoll = 0.03;

float PPitch = PRoll;
float IPitch = IRoll;
float DPitch = DRoll;

float PYaw = 2;
float IYaw = 12;
float DYaw = 0;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
