void battery_voltage(void) {
  Voltage=(float)analogRead(15)/62;
  Current=(float)analogRead(21)*0.089;
}