#include <SPI.h>
#include <Smooth.h>
#include <MozziGuts.h>
#include <string.h>
#include "Notes.h"
#include <Oscil.h> // oscillator template
#include <AutoMap.h>
#include <Smooth.h>
#include <tables/cos2048_int8.h>
#include "Gyro.h"
#include "IMU.h"

#define CONTROL_RATE 256 // powers of 2 please
#define CS_IMU 8
#define ImuToDeg(x) (((float)x)/100.0) // IMU angle info is in deg*100
#define ImuToVel(x) (((float)x)/10.0) // IMU angular vel is in deg/s*10
#define ImuToGee(x) (((float)x)/10000.0)

int roll, pitch, yaw, rolldot, pitchdot, yawdot;
int accel_x, accel_y, accel_z;

Oscil <COS2048_NUM_CELLS, AUDIO_RATE> aCarrier(COS2048_DATA);
Oscil <COS2048_NUM_CELLS, AUDIO_RATE> aModulator(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, CONTROL_RATE> kIntensityMod(COS2048_DATA);

Smooth <long> smoothIntensity(0.9);
Smooth <long> smoothPitch(0.85);
int amplitude = 0;
int baseTone = 0;
long fm_intensity;

void setup()
{
  pinMode(CS_IMU, OUTPUT);
  SPI.begin();
  kIntensityMod.setFreq(4);
  startMozzi(CONTROL_RATE);
}

void loop()
{
  audioHook();
}

int updateAudio()
{
  long modulation = smoothIntensity.next(fm_intensity) * aModulator.next();
  return (aCarrier.phMod(modulation) * amplitude)>>8;
}

void updateControl()
{
  ReadIMU(0);
  int minYaw = -180;
  int maxYaw = 180;
  int shift = 0;

  if(ImuToDeg(pitch) < 0)
  {
    shift = 12;
  }

  baseTone = map(ImuToDeg(yaw), minYaw, maxYaw, 0 + shift, 12 + shift);
  float baseFreq = smoothPitch.next(GetNote(baseTone));
  aCarrier.setFreq(baseFreq);
  aModulator.setFreq(baseFreq);
  fm_intensity = kIntensityMod.next();

  amplitude--;
  
  if(abs(ImuToVel(rolldot)) >= 360) amplitude = 255;
  if(amplitude < 1)
  {
    amplitude = 1;
    fm_intensity = 0;
  }
}

void ReadIMU(byte cmd)
{
  digitalWrite(CS_IMU, LOW); // select device
  byte byte0 = SPI.transfer(cmd); // start it
  roll = getSPIint(cmd);
  pitch = getSPIint(cmd);
  yaw = getSPIint(cmd);
  rolldot = getSPIint(cmd);
  pitchdot = getSPIint(cmd);
  yawdot = getSPIint(cmd);
  accel_x = getSPIint(cmd);
  accel_y = getSPIint(cmd);
  accel_z = getSPIint(cmd);
  digitalWrite(CS_IMU, HIGH); // deselct device
}

int getSPIint(int command)
{
  byte byte0 = SPI.transfer(command);
  byte byte1 = SPI.transfer(command);
  int result = ( ((int)byte1 << 8) | (int)byte0 ); 
  return(result);
}

float GetNote(int semitoneOffset)
{
  return 220 * pow(1.059463, semitoneOffset);
}





