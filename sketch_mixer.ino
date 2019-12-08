// This sketch of Arduino Micro Pro based controller of 2-engine delta wing// airplane. It mixes THR and RUD sygnals and also AIL and ELE sygnals. 
// For input it uses SBUS signal from FrSky S6R receiver connected to 
// Arduino UART (Serial1) port. (The SBUS signal should be inverted). 
// It is possible to use USB-based serial port (Serial) for the debugging. 
// Elecrical schema is in https://s3.eu-central-1.amazonaws.com/andreyr.photos/raznoe/S8RMixer.png

#include <EEPROM.h>

// State variables.
boolean calibOn = false;
boolean startOn = true;
boolean readyOn = false;
boolean evenCycle = false;
volatile unsigned long cycleStart = 0;

// Calibration data EPROM bytes.
const int NUM_PARAMS = 9;
const int RMIN =       0;
const int RMAX =       1;
const int TMIN =       2;
const int TMAX =       3;
const int AMIN =       5;
const int AMAX =       6;
const int EMIN =       7;
const int EMAX =       8;

// RUDDER variables.
int RUD_IN_PIN = 3;
volatile int rIn = 0;
volatile unsigned long rStart = 0;
float rCutA = 0.7;

volatile int tIn = 0;
volatile int aIn = 0;

// ELE variables.
volatile int eIn = 0;

// THR1, THR2 variables.
int THR1_OUT_PIN = 4;
int THR2_OUT_PIN = 5;
volatile int thr1Out = 0;
volatile int thr2Out = 0;

// ELV1, ELV2 variables.
int ELV1_OUT_PIN = 2;
int ELV2_OUT_PIN = 3;
volatile int elv1Out = 0;
volatile int elv2Out = 0;

// LED failsafe and button variables. 
int LED_OUT_PIN = 6;
int BUTTON_IN_PIN = 7;
int FAILSAFE_OUT_PIN = 8;

// SBUS data array.
uint8_t sbusData[25];

float wA = 0;
float wE = 0;
float gain = 0;

void setup(){
  pinMode(0,INPUT_PULLUP);
  // Serial.begin(115200);
  Serial1.begin(100000);
  
  pinMode(THR1_OUT_PIN, OUTPUT);
  pinMode(THR2_OUT_PIN, OUTPUT);
  pinMode(ELV1_OUT_PIN, OUTPUT);
  pinMode(ELV2_OUT_PIN, OUTPUT);
  pinMode(LED_OUT_PIN, OUTPUT); 
  pinMode(FAILSAFE_OUT_PIN, OUTPUT);
    
  pinMode(BUTTON_IN_PIN, INPUT_PULLUP);

  setLED(false);
}

void loop(){
    evenCycle = !evenCycle;
     
    // Sanity check, look if Serial1 isn't empty.
    if (Serial1.available()) {
        readyOn = false;    
        setLED(false);
    }

    // Try to get syncronization wait 1 cycle 
    // collecting all bytes.
    if (!readyOn) {
      delayMicroseconds(100);
      
      int numBytes = Serial1.available();

      if (numBytes) {
        for (int idx=0; idx < numBytes; idx++) 
          Serial1.read();        
      } else {
          readyOn = true;
      }

      return;
    }

    // Read SBUS data, all data should be read in one pack.
    Serial1.readBytes(sbusData, 25);

    cycleStart = micros();

    // Check SBUS data for integrity.                   
    if (sbusData[0] != 0x0f || sbusData[24] != 0) {
      readyOn = false;
      setLED(false);
      return;
    }

    // Blink 3 times, finishing check-up.
    if (startOn) {
      blinkLED(3);
      startOn = false;
      readyOn = false;
      
      return;
    } 

    // Process first 4 channels.
    aIn  = ((sbusData[1]|sbusData[2]<< 8) & 0x07FF);
    eIn  = ((sbusData[2]>>3|sbusData[3]<<5) & 0x07FF);
    tIn  = ((sbusData[3]>>6|sbusData[4]<<2|sbusData[5]<<10) & 0x07FF);
    rIn  = ((sbusData[5]>>1|sbusData[6]<<7) & 0x07FF);

    boolean failsafeOn = sbusData[23] & (1<<3);
    digitalWrite(FAILSAFE_OUT_PIN, failsafeOn ? HIGH : LOW);
    
    // THR and RUD mix calculation.
    float rInA = getValue(rIn);
    float tInA = getValue(tIn);
    float diffCorr = 0.8;   
    
    float diffRA = 2.0 * (rInA - 0.5);    
    float thr1 = tInA * (1.0 + diffRA * diffCorr);
    float thr2 = tInA * (1.0 - diffRA * diffCorr);

    float extra1 = 0;
    float extra2 = 0; 
    float dummy = 0;        
    
    if (thr1 > 1.0) 
      extra1 = thr1 - 1.0;
    else 
      dummy = thr1 - 1.0;
      
    if (thr2 > 1.0) 
      extra2 = thr2 - 1.0;
    else 
      dummy = thr1 - 1.0;

    // AIL and ELE calcualtaion.
    float aInA = getValue(aIn);
    float eInA = getValue(eIn);
    
    float ail1 = (eInA + aInA) / 2.0;
    float ail2 = (eInA + (1 - aInA)) / 2.0; 
           
    thr1Out = setValue(thr1 - extra2);
    thr2Out = setValue(thr2 - extra1);   
    elv1Out = setGValue(ail1);
    elv2Out = setGValue(1.0 - ail2);

    // Do output 50 Hz generation.
    // SBUS cycle frequency is 100 Hz use 
    // even and odd cycles for AIL and THR.
    if (!calibOn) {      
      waitTimePoint(cycleStart + 1000);    
             
      if (evenCycle) {
          doPulse(ELV1_OUT_PIN, elv1Out);  
      } else {
          doPulse(THR1_OUT_PIN, thr1Out);
      }
  
      waitTimePoint(cycleStart + 4000);    
             
      if (evenCycle) {
          doPulse(ELV2_OUT_PIN, elv2Out);  
      } else {
          doPulse(THR2_OUT_PIN, thr2Out);
      }
    }   
}

// UTILITY FUNCTIONS

// Converts PPM pulse length in mks
// to double value in the range [0.0 - 1.0].
float getValue(int v)
{
  return (v - 470) * 1.0 / (1520 - 470);
}

// Converts double value [0.0 - 1.0] to 
// the PPM pulse length in mks.
int setValue(float v)
{

  if (v < 0) v = 0;
  if (v > 1) v = 1;
  
  // This are experimentally set coefficients 
  // for transfer SBUS value to pulse length.
  return 950 + v * 1100;
}

// Converts double value [0.0 - 1.0] to 
// the PPM pulse length in mks.
int setGValue(float v)
{
  v = (v - 0.5) * 2.0 + 0.5;
  
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  
  // This are experimentally set coefficients 
  // for transfer SBUS value to pulse length.
  return 950 + v * 1100;
}

// Blinks LED num times.
void blinkLED(int num)
{
  for (int idx=0; idx<num; idx++) {
    setLED(true);
    delay(150);    
    setLED(false);
    delay(150); 
    setLED(true);     
  }
}

// Sets LED on.
void setLED(boolean isOn)
{
  digitalWrite(LED_OUT_PIN, isOn ? 1 : 0);    
}

// Test if the button is pressed.
boolean buttonPressed() 
{
  return ! digitalRead(BUTTON_IN_PIN);
}

void waitTimePoint(unsigned long point)
{
  int delay = point - micros();

  if (delay > 0) {
    delayMicroseconds(delay);
  }   
}
  
void doPulse(int pin, int pulseWidth)
{      
    digitalWrite(pin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(pin, LOW);    
}
