#include <Wire.h>
float RateRoll, RatePitch, RateYaw, AccX, AccY, AccZ;
float RateCaliRoll, RateCaliPitch, RateCaliYaw;
float Roll, Pitch, Yaw;
int RateCaliNumber;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;

float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[2] = {0,0};
float svAngleRoll, svAnglePitch;

void Kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasure){
  KalmanState = KalmanState + 0.004*KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004*0.004*4*4;
  float KalmanGain = KalmanUncertainty/(KalmanUncertainty+(3*3));
  KalmanState = KalmanState + KalmanGain*(KalmanMeasure-KalmanState);
  KalmanUncertainty = (1-KalmanGain)*KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();


  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB=Wire.read()<<8 | Wire.read();
  int16_t AccYLSB=Wire.read()<<8 | Wire.read();
  int16_t AccZLSB=Wire.read()<<8 | Wire.read();

  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;

  AccX = (float)AccXLSB/4096;
  AccY = (float)AccYLSB/4096;
  AccZ = (float)AccZLSB/4096;

  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))* (180/3.142);
  AnglePitch = -atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))* (180/3.142);

}

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // calibration gyroscope

  for(int i =0; i<2000; i++){
      gyro_signals();
      RateCaliRoll += RateRoll;
      RateCaliPitch += RatePitch;
      RateCaliYaw += RateYaw;
      delay(1);
  }
  RateCaliRoll = RateCaliRoll/2000;
  RateCaliPitch = RateCaliPitch/2000;
  RateCaliYaw = RateCaliYaw/2000;
  gyro_signals();
  Roll -= RateRoll;
  Pitch -= RatePitch;
  svAnglePitch = Pitch;
  svAngleRoll = Roll;
  LoopTimer = micros();
}

void loop() {
  gyro_signals();

//  Roll -= RateRoll;
//  Pitch -= RatePitch;
//  Serial.print("Roll Angle: ");
//  Serial.print(Roll);
//
//  Serial.print(" Pitch Angle: ");
//  Serial.println(Pitch);

  RateRoll -=RateCaliRoll;
  RatePitch -= RateCaliPitch;
  RateYaw -= RateCaliYaw;

  Kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  Kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
  Serial.print("Roll Angle: ");
  Serial.print(KalmanAngleRoll);

  Serial.print(" Pitch Angle: ");
  Serial.println(KalmanAnglePitch);

  while(micros() - LoopTimer < 4000);
  LoopTimer = micros();

}