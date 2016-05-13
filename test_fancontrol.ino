/**
 * Test-harness for the fancontrol PCB.
 * 
 * Works like this:
 * - Mute flashes, and the polarity reverses when REMOTE is asserted
 * - Fan is on when REMOTE is asserted
 * - each rising edge of REMOTE toggles the fan speed between high & low
 * - every second rising edge of REMOTE toggles the RELAY pin
 * - DS1621s are programmed for 30/28C hard-shutdown of RELAY pin (hand-warm to test)
 * - FAULT LED is poll-copied from the PWM pin so will flash when fan runs slowly
 */

#include <Arduino.h>
#include <Wire.h>
#include <DS1621.h>

#define DS_SDA A4   ///< fixed pins defined by I2C hardware
#define DS_SCL A5

#define FAULT  13     ///< indicator LED
#define RELAY 5       ///< power relay to run amplifiers
#define FAN  11       ///< timer2 HF PWM output  (FIXED PIN)
#define FANPWR 10     ///< fan power-up
#define REMOTE 3      ///< remote-powerup input
#define SPEED 8       ///< ICP1=PB0=8, tacho input (FIXED PIN)
#define MUTE 6        ///< mute signal for de-thump
#define IR 2          ///< infrared-remote

#define SENSOR_MASK 0x03
#define SENSOR_CFG DS1621::CFG_POL

const double over_temp=30.0;      ///< thermostat hard-off
const double high_temp=28.0;      ///< thermostat enable
const short hw_shutdown=DS1621::tempFromDegC(over_temp);   ///< hardware power-down on software failure
const short hw_recover=DS1621::tempFromDegC(high_temp);    ///< point at which hardware power-down releases

// output pulse params
#define PHASES 2
const unsigned long times[PHASES]={ 50, 200 };
const unsigned long period=250;

DS1621 sensor[8]={
  DS1621(0), DS1621(1), DS1621(2), DS1621(3),
  DS1621(4), DS1621(5), DS1621(6), DS1621(7)
};

unsigned long lasttime=0, lastrise=0;
int phase=0, fanmode=0;
bool oldrem;

void setup() {
  // put your setup code here, to run once:

  pinMode(FAULT, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(FANPWR, OUTPUT);
  pinMode(MUTE, OUTPUT);
  pinMode(REMOTE, INPUT);
  pinMode(SPEED, INPUT);
  pinMode(IR, INPUT);

  digitalWrite(FAULT, 0);
  digitalWrite(RELAY, 0);
  digitalWrite(FAN, 0);
  digitalWrite(FANPWR, 0);
  digitalWrite(MUTE, 0);

  Wire.begin();
  for(int i=0;i<8;++i){
    if((SENSOR_MASK & (1<<i)) != 0){
      sensor[i].setConfig(SENSOR_CFG);
      sensor[i].convert();
      sensor[i].setTH(hw_shutdown);
      sensor[i].setTL(hw_recover);
    }
  }

  lasttime=millis();
  oldrem=!digitalRead(REMOTE);
}

void loop() {
  
  unsigned long now=millis();
  unsigned long dt=now-lasttime;

  if(dt > times[phase]){
    lasttime+=times[phase++];
    if(phase >= PHASES){
      phase=0;
    }
  }

  bool rem=!digitalRead(REMOTE);
  if(rem && !oldrem && (now-lastrise > 100)){
    lastrise=now;
    ++fanmode;

    digitalWrite(FAN, (fanmode & 0x01) != 0);
    digitalWrite(RELAY, (fanmode & 0x02) != 0);
  }
  digitalWrite(FANPWR, rem);
  oldrem=rem;

  digitalWrite(FAULT, digitalRead(SPEED));

  switch(phase){
  case 0:
    digitalWrite(MUTE, rem);
    break;
  case 1:
    digitalWrite(MUTE, !rem);
    break;
  }
  
}
