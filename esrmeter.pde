#include <Wire.h>

#include <EEPROM.h>
//#include <LiquidTWI2.h>
#include <LiquidCrystal.h>
#include "EEPROMAnything.h"

//we have to change prescaler for the ADC to make the conversion happen faster
//this code section was suggested on the arduino forum
#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//define the input and output pin we will use
#define DISCHARGE_PIN 19
#define ESR_PIN A0
#define PULSE_SMALL 17 //this is not used in the sketch, implement it as needed (for 5mA current needed for smaller cap value measurement)
#define PULSE_PIN 16
#define BUTTON_PIN 24

//function prototype
unsigned long measureESR(void);//measuring function, increases ADC to 16bit resolution through oversampling

//global variables
unsigned long esrSamples;
double miliVolt;
double esrVal;
double esrCal;
double vRef = 3.31;//voltage on the Vref pin (this sketch uses internal voltage reference 1.1V)
double current = 0.02468;//0.046200;//proper calibration can be done entering the right value for the current (U=I*R)
//idealy this is 0.05 A, this condition is fulfilled only if R10 is 100 Ohm, Vcc is exactly 5V and the transistor
//while fully saturated idealy is at 0 ohm.

LiquidCrystal lcd(13,15,20,21,22,23);

void setup(void)
{
 // lcd.setMCPType(LTI_TYPE_MCP23017);
 
 pinMode(10,OUTPUT);
 digitalWrite(10,LOW);

 pinMode(12,OUTPUT);
 digitalWrite(12,LOW);
 
 pinMode(14,OUTPUT);
 digitalWrite(14,LOW);
 
 
  lcd.begin(16,2);
  
  lcd.setCursor(0,0);
  lcd.print("ESR meter");
  lcd.setCursor(5,1);
  lcd.print("version 0.1");
  delay(500);

  lcd.print("Seting up...");

  analogReference(DEFAULT);//setting vRef to internal reference 1.1V

  pinMode(ESR_PIN, INPUT);//reading miliVolt
  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN,HIGH);//low enables T1
  pinMode(DISCHARGE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN,HIGH);//low disables T2
  pinMode(BUTTON_PIN,INPUT_PULLUP);//setting up for a button (will use this for zeroing)

  delay(500);
  lcd.clear();
  lcd.print("Please Wait...");

  //reading calibration value, it will be ok if already calibrated, else it might be bogus depends on the content of EEPROM
  //but will be ok after first calibration
    EEPROM_readAnything(0,esrCal);
}

void loop(void)
{
  esrSamples = measureESR();//this function takes a while,)
  // so we don't need other delay for the lcd (this functions time gives the refresh rate for display
  miliVolt = (esrSamples * vRef) / 65.535;//calculating voltage on AIN0 pin
  esrVal = (miliVolt)/current - esrCal;//calculate ESR in miliOhm (pls read forum for correct formula)

  lcd.clear();
  lcd.print("  V:");
  lcd.print(miliVolt,4);
  lcd.setCursor(13,0);  
  lcd.print("mV");
  lcd.setCursor(0,1);
  lcd.print("ESR:");
  lcd.print(esrVal,4);
  lcd.setCursor(13,1);
  lcd.print("m");
  lcd.print((char)244);

  //for zeroing the cables, this can be quite a big resistance compared to the values we intend to measure
  //so it is a good idea to try to reduce in any way possible this influence (short cables, soldering the cables, etc)
  if(!digitalRead(BUTTON_PIN)){
    lcd.clear();
    lcd.print("Zeroing...");
    esrCal = (miliVolt)/current;
    lcd.print(" done!");
    lcd.setCursor(0,1);
    //writing calibration value into EEPROM so we don't have to calibrate on restart
    EEPROM_writeAnything(0,esrCal);
    lcd.print("saved to EEPROM");
    delay(400);
  }
}

//this is where the magic happens, it really works and gives some
//incredibly good results! if you need sub milivolt accuracy is a good way to go
//noise is good ;) if in doubt must read oversampling on ADC from AVR docs
unsigned long measureESR()
{
  unsigned long samples = 0;
  unsigned int acumulator = 0;
  int i = 0;
  //oversampling 4096 times (for 16 bit is 4^(desiredResolution - ADCresolution))
  while(i < 4096) {
    digitalWrite(DISCHARGE_PIN,LOW);//disable discharging
    digitalWrite(PULSE_PIN,LOW);//making a miliVolt pulse of 50mA
    delayMicroseconds(1);//on the scope it looks that after enabling the pulse a litle delay is
    //recomended so the oscillations fade away
    acumulator = analogRead(ESR_PIN);//reading value on AIN0
    digitalWrite(PULSE_PIN,HIGH);//stopping pulse
    digitalWrite(DISCHARGE_PIN,HIGH);//discharging the capacitors
    delayMicroseconds(600);//waiting a bit longer to fully discharge before another pulse
    samples += acumulator;//acumulating the readings
    i++;
  }
  //we have samples, let's go and compute value
  samples = samples >> 6;//decimating value
  return samples;//all done returning sampled value
}

