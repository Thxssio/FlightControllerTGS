void configSetup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for (CalibrationNumber = 0; 
       CalibrationNumber < 2000; 
       CalibrationNumber++) {
    gyro_signals();

    CalibrationRoll += Roll;
    CalibrationPitch += Pitch;
    CalibrationYaw += Yaw;
    delay(1);
    
  }

  CalibrationRoll /= 2000;
  CalibrationPitch /= 2000;
  CalibrationYaw /= 2000;


  analogWriteFrequency(1, 250);
  analogWriteFrequency(2, 250);
  analogWriteFrequency(3, 250);
  analogWriteFrequency(4, 250);

  analogWriteResolution(12);

  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  battery_voltage();
  if (Voltage > 8.3) {
    digitalWrite(5, LOW);
    BatteryStart = BatteryDefault;
  } else if (Voltage < 7.5) {
    BatteryStart = 30 / 100 * BatteryDefault;
  } else {
    digitalWrite(5, LOW);
    BatteryStart = (82 * Voltage - 580) / 100 * BatteryDefault;
  }
  ReceiverInput.begin(14);
  while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050) {
    read_receiver();
    delay(4);
  }
  LoopTimer = micros();
}