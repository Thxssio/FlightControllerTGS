//PID Eq

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > 400) Iterm = 400;
  else if (Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError) / 0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400) PIDOutput = 400;
  else if (PIDOutput < -400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}
void reset_pid(void) {
  PrevErrorRoll = 0;
  PrevErrorPitch = 0;
  PrevErrorYaw = 0;
  PrevItermRoll = 0;
  PrevItermPitch = 0;
  PrevItermYaw = 0;
}