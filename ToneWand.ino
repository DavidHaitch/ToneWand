#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <Smooth.h>
#include <MozziGuts.h>
#include <Oscil.h>
#include <AutoMap.h>
#include <Smooth.h>
#include <tables/sin2048_int8.h>
#include <tables/saw2048_int8.h>

#define NEOPIXEL_PIN 4

#define CONTROL_RATE 128 // powers of 2 please
#define CS_IMU 8
#define ImuToDeg(x) (((float)x)/100.0) // IMU angle info is in deg*100
#define ImuToVel(x) (((float)x)/10.0) // IMU angular vel is in deg/s*10
#define ImuToGee(x) (((float)x)/10000.0)
#define FALLOFF 2
#define SCALE 8
int roll, pitch, yaw, rolldot, pitchdot, yawdot;
int accel_x, accel_y, accel_z;

Oscil <SAW2048_NUM_CELLS, AUDIO_RATE> aCarrier(SAW2048_DATA);
Oscil <SIN2048_NUM_CELLS, AUDIO_RATE> aHarmonic(SIN2048_DATA);
Smooth <long> smoothPitch(0.85);

int amplitude = 0;
int baseTone = 0;
float scaleCoefficient;
float counter = 0;

Adafruit_NeoPixel ring = Adafruit_NeoPixel(16, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800);
void setup()
{
  scaleCoefficient = pow(2, 1.0/SCALE);
  pinMode(CS_IMU, OUTPUT);
  SPI.begin();
  startMozzi(CONTROL_RATE);

  ring.begin();
  ring.setBrightness(32); //0 - 255 scale
  for(int i = 0; i < 16; i++)
  {
    //ring.setPixelColor(i, Wheel(i*16));
  }

  //ring.show();
}

void loop()
{
  audioHook();
}

int updateAudio()
{
  return (aCarrier.phMod(aHarmonic.next()) * amplitude)>>8;
}

void updateControl()
{
  ReadIMU(0);
  int minYaw = -180;
  int maxYaw = 180;
  int shift = 0;


  baseTone = map(ImuToDeg(yaw), minYaw, maxYaw, 0, SCALE * 2);
  if(abs(ImuToDeg(pitch)) <= 10)
  {
    amplitude = 255;
    float baseFreq = GetNote(baseTone);
    aCarrier.setFreq(baseFreq);
    aHarmonic.setFreq(GetNote(baseTone - 48));
  }
  else
  {
    amplitude -= FALLOFF;
  }

  if(amplitude <= 32)
  {
    UpdateLights((baseTone % 16) + 1);
  }

  if(amplitude < 0) amplitude = 0;
}


int counter2 = 0;
void UpdateLights(int param)
{
  for(int i = 0; i < param; i++)
  {
    ring.setPixelColor((i+5)%16, Wheel(((i * 256 / ring.numPixels()) + counter2) & 255));
  }

  if(ImuToDeg(pitch) > 20)
  {
    counter2 += 2;
  }
  else
  {
    counter2 -= 2;
  }
  ring.show();
  for(int i = 0; i < param; i++)
  {
    ring.setPixelColor((i+5)%16, 0);
  }

  counter+= 0.005;
  if(counter > 3.14) counter = 0;
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return ring.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return ring.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return ring.Color(0, WheelPos * 3, 255 - WheelPos * 3);
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
  return 110 * pow(scaleCoefficient, semitoneOffset);
}






