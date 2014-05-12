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

float smoothness = 0.95;
Smooth <long> smoothIntensity(smoothness);

int amplitude = 0;
int baseTone = 0;
int mod_ratio = 6; // brightness (harmonics)
long fm_intensity;

void setup()
{
  Serial.begin(57600);
  pinMode(CS_IMU, OUTPUT);
  SPI.begin();
  
  kIntensityMod.setFreq(500);
  startMozzi(CONTROL_RATE); // set a control rate of 64 (powers of 2 please)
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

int amplitudeMod = 0;
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
  if(abs(ImuToVel(rolldot)) >= 360)
  {
    amplitude = 255;
  }

  float baseFreq = GetNote(baseTone);
  int mod_freq = baseFreq * mod_ratio;
  aCarrier.setFreq(baseFreq);
  aModulator.setFreq(mod_freq);
  fm_intensity = (100 * (kIntensityMod.next()+128))>>8; // shift back to range after 8 bit multiply

  amplitude -= 3;
  if(amplitude > 255) amplitude = 255;
  if(amplitude < 0) amplitude = 0;
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




