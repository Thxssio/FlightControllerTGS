void imu() {

  gyro_signals();

  Roll -= CalibrationRoll;
  Pitch -= CalibrationPitch;
  Yaw -= CalibrationYaw;

  read_receiver();

  RollDesired = 0.15 * (ReceiverValue[0] - 1500);
  PitchDesired = 0.15 * (ReceiverValue[1] - 1500);

  InputThrottle = ReceiverValue[2];
  YawDesired = 0.15 * (ReceiverValue[3] - 1500);

  ErrorRoll = RollDesired - Roll;
  ErrorPitch = PitchDesired - Pitch;
  ErrorYaw = YawDesired - Yaw;

  pid_equation(
    ErrorRoll,
    PRoll,
    IRoll,
    DRoll,
    PrevErrorRoll,
    PrevItermRoll);

  InputRoll = PIDReturn[0];
  PrevErrorRoll = PIDReturn[1];
  PrevItermRoll = PIDReturn[2];

  pid_equation(
    ErrorPitch,
    PPitch,
    IPitch,
    DPitch,
    PrevErrorPitch,
    PrevItermPitch);

  InputPitch = PIDReturn[0];
  PrevErrorPitch = PIDReturn[1];
  PrevItermPitch = PIDReturn[2];

  pid_equation(
    ErrorYaw,
    PYaw,
    IYaw,
    DYaw,
    PrevErrorYaw,
    PrevItermYaw);

  InputYaw = PIDReturn[0];
  PrevErrorYaw = PIDReturn[1];
  PrevItermYaw = PIDReturn[2];

  if (InputThrottle > 1800) InputThrottle = 1800;
  MotorInput1 = 1.024 * (InputThrottle - InputRoll - InputPitch - InputYaw);
  MotorInput2 = 1.024 * (InputThrottle - InputRoll + InputPitch + InputYaw);
  MotorInput3 = 1.024 * (InputThrottle + InputRoll + InputPitch - InputYaw);
  MotorInput4 = 1.024 * (InputThrottle + InputRoll - InputPitch + InputYaw);
  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999;
  if (MotorInput3 > 2000) MotorInput3 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;
  int ThrottleIdle = 1180;
  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;
  int ThrottleCutOff = 1000;

  if (ReceiverValue[2] < 1050) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;
    reset_pid();
  }

  analogWrite(1, MotorInput1);
  analogWrite(2, MotorInput2);
  analogWrite(3, MotorInput3);
  analogWrite(4, MotorInput4);
  battery_voltage();

  CurrentConsumed = Current * 1000 * 0.004 / 3600 + CurrentConsumed;
  BatteryLevel = (BatteryStart - CurrentConsumed) / BatteryDefault * 100;

  if (BatteryLevel <= 30) digitalWrite(5, HIGH);
  else digitalWrite(5, LOW);
  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();
}
