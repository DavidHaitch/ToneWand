#include <SPI.h>
#include <Smooth.h>
#define CS_IMU 8 // Chip select for IMU can be any open digital pin
#define ImuToDeg(x) (((float)x)/100.0) // IMU angle info is in deg*100
#define ImuToVel(x) (((float)x)/10.0) // IMU angular vel is in deg/s*10
#define ImuToGee(x) (((float)x)/10000.0)
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
int roll, pitch, yaw, rolldot, pitchdot, yawdot;
int accel_x, accel_y, accel_z;
float minYaw = -10.0;
float maxYaw = 10.0;
int lastTone = 1000;
Oscil<COS2048_NUM_CELLS, AUDIO_RATE> aCos(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, AUDIO_RATE> aChord1(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, AUDIO_RATE> aChord2(COS2048_DATA);
Smooth <int> kSmoothNote(0.8);
int amplitude = 0;
int lastAmplitude;
int baseTone = 0;
void setup()
{
  Serial.begin(57600);
  pinMode(CS_IMU, OUTPUT);
  SPI.begin();

  startMozzi(CONTROL_RATE); // set a control rate of 64 (powers of 2 please)
}

void loop()
{
  audioHook();
}

int updateAudio()
{
    int note = aCos.next();
    return (note * amplitude)>>8; // phase modulation to modulate frequency
}

int amplitudeMod = 0;
void updateControl()
{
  ReadIMU(0);
  
  
  minYaw = -180;
  maxYaw = 180;
  
  int shift = 0;
  
  if(ImuToDeg(pitch) < 0)
  {
    shift = 12;
  }
  
  baseTone = map(ImuToDeg(yaw), minYaw, maxYaw, 0 + shift, 12 + shift);

  //amplitude = map(abs(ImuToDeg(roll)), 0, 90, 0, 255);
  
  if(abs(ImuToVel(rolldot)) >= 360)
  {
    amplitude = 255;
  }

  lastTone = baseTone;
  float baseFreq = GetNote(baseTone);
  aCos.setFreq(kSmoothNote.next(baseFreq));
  //aChord2.setFreq(GetNote(baseTone+4));
  lastAmplitude = amplitude;
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




