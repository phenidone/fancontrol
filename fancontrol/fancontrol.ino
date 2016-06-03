/**
 * Amplifier Fan Controller
 * (C) 2014-2016 William Brodie-Tyrrell
 *
 * This is a PID-based fan-speed controller for use in conjunction with an audio power
 * amplifier or similar linear device.  It has two temp-control modes:
 *  - high-temp (e.g. 45C) quiet mode
 *  - low-temp, high-power mode (e.g. 30C)
 *
 * It defaults to high-temp mode at startup, keeping the fan as quiet as possible while
 * preventing the amp from getting problematically hot.  If the user cranks the amplifier power,
 * fan speed will be seen climbing in order to maintain temperature.  In that case, the 
 * temperature set-point will be brought down and the fan will run harder to keep the system
 * cooler while operating at a higher power level.  The purpose is to match the thermal conditions
 * to the load:
 *  - at low power, higher temp is acceptable but fan noise is not,
 *  - at high power, the fan is inaudible and lower temp is desirable to maximise transistor SOA.
 *
 * There are two nested PID loops:
 *  - temperature setpoint, the output of which is fan RPM, and
 *  - fan speed, the output of which is a PWM signal
 *
 * The inner (fan speed) loop runs at ~5Hz and the outer (temperature) loop runs at ~0.25Hz.
 *
 * Amplifier power is applied only when:
 *  - the remote-trigger signal is asserted,
 *  - the fan has not stalled, 
 *  - the AVR is successfully getting temp readings from all sensors, and
 *  - all measured temperatures are below the safety-cutoff level
 *      - hard backup: temp sensors have an "overtemp" signal that will cut power even
 *        in the case of software failure.
 *
 * There are the following modes, treated as a finite state machine:
 * - off (remote signal is off)
 * - init (full fan power at startup to overcome stiction; lasts 1s)
 * - cooldown (remote off, but temp still high)
 * - delay (remote has turned on, we are waiting before turning on the power)
 * - mute (amp turned on but held in mute while its power supply stabilises)
 * - run (normal operation)
 * - failed (overheat or fan dead)
 *
 * When entering failure-mode, the fault-LED will flash a few times:
 * 1: overheat
 * 2: fan stall
 * 3: sensor failure
 *
 * Future work and missing features
 *  - AC detection
 *  - low-temp fan power-down, independent of relay
 *  - IR remote receiver
 *  - put control-parameters in EEPROM and permit changes over serial
 *
 * Intended target is an Arduino Nano v3, with custom carrier/shield board.
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
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

#define SENSOR_MASK 0x03   ///< 8-bit mask of which DS1621 addresses have devices on them
                           // for example if 0, 1 and 7 are present, it is 0x83

/// configuration bits for sensors.  Active-high, not 1-shot.
#define SENSOR_CFG DS1621::CFG_POL

// LED flash-codes for failures
#define CODE_OVERHEAT 1
#define CODE_FANSTALL 2
#define CODE_SENSOR   3

/// temp-sensor objects.
DS1621 sensor[8]={
  DS1621(0), DS1621(1), DS1621(2), DS1621(3),
  DS1621(4), DS1621(5), DS1621(6), DS1621(7)
};

// control presets
const double over_temp=60.0;       ///< cooking, turn it all off.
const double high_temp=50.0;       ///< high-temp/low-speed preset
const double low_temp=40.0;        ///< low-temp/high-speed preset
const short hw_shutdown=DS1621::tempFromDegC(over_temp);   ///< hardware power-down on software failure
const short hw_recover=DS1621::tempFromDegC(high_temp);    ///< point at which hardware power-down releases
const double threshold_up=1500;    ///< if fan faster than this, lower the preset
const double threshold_down=1200;  ///< if fan slower than this, raise the preset
const double rpm_min=450, rpm_max=2000;  ///< fan capabilities/behaviour
const double rpm_stall=0.5*rpm_min;
const long threshold_time=60000;   ///< speed must cross threshold for 1 minute to change temp preset
const long init_delay=1000;        ///< fan test-spinup time at boot
const long on_delay=500;           ///< wait this long after remote asserted for fan to spin-up before applying amp power
const long mute_delay=200;         ///< wait this long before coming out of mute

// timer speed that we clock the tacho with
const float timer_rpm=7.5e6;
// PID sample periods
const int Ts_rpm=200, Ts_temp=4000;
// PID state-variables 
double fanrpm=0.0, fanpwm=255.0, rpmSetpoint=rpm_max;
double temperature=25.0, tempSetpoint=high_temp;

/// sensor-failure
bool failflag=false;
/// fan SHOULD be running, so can fail if this is set but it's stalled
bool checkfan=false;

/// inner control loop
PID rpmpid(&fanrpm, &fanpwm, &rpmSetpoint, 0.02, 0.05, 0, DIRECT);
/// outer control loop
PID temppid(&temperature, &rpmSetpoint, &tempSetpoint, 50, 25, 0, REVERSE);

/// call this to move into a different mode
void changeMode(int mode);

/// poll-function in init mode
void poll_init();
/// poll-function in off mode
void poll_off();
/// poll-function in cooldown mode
void poll_cooldown();
/// poll-function in delay mode
void poll_delay();
/// poll-function in mute mode
void poll_mute();
/// poll-function in run mode
void poll_run();
/// poll-function in failed mode
void poll_failed();
/// mode-entry function in init mode
void enter_init();
/// mode-entry function in off mode
void enter_off();
/// mode-entry function in cooldown mode
void enter_cooldown();
/// mode-entry function in delay mode
void enter_delay();
/// mode-entry function in mute mode
void enter_mute();
/// mode-entry function in run mode
void enter_run();
/// mode-entry function in failed mode
void enter_failed();

/// operational modes
enum {
  MODE_INIT,      ///< AVR initialisation, run fan up to speed
  MODE_OFF,       ///< remote is off, amp and fan are off
  MODE_COOLDOWN,  ///< remote and amp off/muted, fan running until cooler
  MODE_DELAY,     ///< power-on sequence delay: remote, amp still off
  MODE_MUTE,      ///< power-on mute: amp on but muted
  MODE_RUN,       ///< normal run
  MODE_FAILED,    ///< failed, amp off and fan on full until cooled
  MODE_COUNT      ///< number of modes
};

/// state-machine current-state indicator
int currentMode=MODE_INIT;
typedef void (*modefunc)();
/// function pointers for polling functions in each state
modefunc pollfuncs[MODE_COUNT]
= { 
  poll_init, poll_off, poll_cooldown, poll_delay,
  poll_mute, poll_run, poll_failed };
/// function pointers for mode-entry functions for each state
modefunc enterfuncs[MODE_COUNT]
= { 
  enter_init, enter_off, enter_cooldown, enter_delay,
  enter_mute, enter_run, enter_failed };

/// when the last mode-change occurred
unsigned long modeChangeTime;
/// when the last speed/temp measurements were performed
unsigned long measurementTime;
/// delay time until next poll cycle
int nextPoll=20;

// last N measurements, for discarding outliers (RPM)
const int periodWindowBits=3;
const int periodWindowLength=1<<periodWindowBits;
const int periodWindowMask=periodWindowLength-1;
long periodWindow[periodWindowLength];
long periodWindowPtr=-1;

/// track timer overflows
short accumPeriod=0;
/// don't accumulate longer than this
const short maxAccum=1;

// user-space code uses this to observe changes in periodWindowPtr;
// not used in interrupt code
long prevPeriodPtr=-1;

// for debug and status messages
char printbuf[64];

/// setup timer1 to count fan pulses
void speedSetup()
{ 
  pinMode(SPEED, INPUT);
  digitalWrite(SPEED, 1);

  for(char k=0;k<periodWindowLength;++k)
    periodWindow[k]=0;

  // period measurement (input capture)
  noInterrupts();
  TCCR1A=0x00;    // normal running timer
  TCCR1B=0xC3;    // noise cancel, rising edge, CLK/64
  TIMSK1=0x21;    // input capture and overflow interrupts
  interrupts();
}

/// fan rpm timer overflow
ISR(TIMER1_OVF_vect)
{
  ++accumPeriod;
}

/// fan rpm input capture, i.e. tacho leading edge
ISR(TIMER1_CAPT_vect)
{
  TCNT1=0;  // clear timer

  // is not an invalidly long time, record it
  if(accumPeriod <= maxAccum){
    // current timer plus all previous overflows
    long lastPeriod=(long(accumPeriod)<<16)+ICR1;
    // stash the value in the buffer and move ptr along
    periodWindow[++periodWindowPtr & periodWindowMask]=lastPeriod;
  }

  accumPeriod=0;  
}

const float outlierExclude=2.0;

/// return mean value excluding outliers
/// need to compute median instead: outliers are perturbing mean so far as
/// to exclude all valid data
float filterPeriod(long *window, char *count, char startat, char maxsamp)
{
  startat=startat-maxsamp+1+periodWindowLength;

  float mean=0;
  for(char k=0;k<maxsamp;++k){
    long val=window[(startat+k) & periodWindowMask];
    mean+=val;
    // Serial.print(val); Serial.print(" ");
  }
  // Serial.println("");

  float exclMin=mean/(maxsamp*outlierExclude);
  float exclMax=(outlierExclude*mean)/maxsamp;
  mean=0;

  *count=0;
  for(char k=0;k<maxsamp;++k){
    long val=window[(startat+k) & periodWindowMask];
    if(val > exclMin && val < exclMax){
      // Serial.print(val); Serial.print(" ");
      ++*count;
      mean+=val; 
    }
  }
  // Serial.println("");

  return mean/count[0];
}


/// setup Timer2 as high-speed (32kHz) PWM
void pwmSetup()
{
  OCR2A=0xFF;    // max output
  TCCR2A=0x81;   // phase-correct PWM, non-inverted
  TCCR2B=0x01;   // no prescale, 32kHz output
}

/// setup all the expected temperature sensors
void tempSetup()
{
  Wire.begin();

  sprintf(printbuf, "TH/TL: %04X %04X\n", hw_shutdown, hw_recover);
  Serial.print(printbuf);

  // pre-configure all sensors that we expect to be present
  for(int i=0;i<8;++i){
    if((SENSOR_MASK & (1<<i)) != 0){
      sensor[i].setConfig(SENSOR_CFG);
      sensor[i].convert();
      sensor[i].setTH(hw_shutdown);
      sensor[i].setTL(hw_recover);
    }
  }
}

/// monitor all the temperature sensors
void tempMeasure()
{
  short maxtmp=DS1621::INVALID_TEMP;
  failflag=false;

  // inspect all expected sensors
  for(int i=0;i<8;++i){
    if((SENSOR_MASK & (1<<i)) != 0){

      Serial.print("[");
      Serial.print(i);
      Serial.print("] ");

      // check that sensor is present and reporting correct configuration
      if((sensor[i].getConfig() & DS1621::CFG_CFG) != SENSOR_CFG ||
          sensor[i].getTL() != hw_recover || 
          sensor[i].getTH() != hw_shutdown){
        sprintf(printbuf, "%02X %02X %02X\n", sensor[i].getConfig(), sensor[i].getTL(), sensor[i].getTH());
        Serial.print(printbuf);
        Serial.print("not present or misconfigured\n");
        failflag=true;
        continue;
      }
      
      short tmp=sensor[i].getTemp();

      // sensor unreachable -> FAIL!
      if(tmp == DS1621::INVALID_TEMP){
        Serial.print(" FAILED\n");
        failflag=true;
        continue;
      }

      Serial.print(" = ");
      Serial.print(DS1621::tempToDegC(tmp));
      Serial.print("\n");
      // kick it along, even if it forgot it's not meant to be 1-shot.
      sensor[i].convert();

      // use the highest reading
      if(tmp > maxtmp){
        maxtmp=tmp;
      }
    }
  }

  // convert to floating-point for PID input.
  temperature=DS1621::tempToDegC(maxtmp);
}

/// main entry-point: configure the system
void setup()
{  
  // boot & configure the temp sensors
  Serial.begin(9600);
  tempSetup();

  // setup RPM measurement
  speedSetup();

  // configure PIDs
  rpmpid.SetSampleTime(Ts_rpm);
  rpmpid.SetOutputLimits(0, 255);  // PWM bounds
  rpmpid.SetMode(AUTOMATIC);

  temppid.SetSampleTime(Ts_temp);
  temppid.SetOutputLimits(rpm_min, rpm_max);  // fan speed range
  temppid.SetMode(AUTOMATIC);
  tempSetpoint=high_temp;  

  // setup other IO to a nice default state
  digitalWrite(FANPWR, 0);
  digitalWrite(FAN, 0);
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FAULT, 0);
  pinMode(FAN, OUTPUT);
  pinMode(FANPWR, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(FAULT, OUTPUT);
  pinMode(MUTE, OUTPUT);
  pinMode(REMOTE, INPUT);
  pinMode(IR, INPUT);
  pwmSetup();

  // we startup and wait for the fan to get going
  changeMode(MODE_INIT);
}

/// enter failure-mode and display N flashes on the fault LED.
void indicateFailure(unsigned count)
{
  changeMode(MODE_FAILED);

  for(unsigned i=0;i<count;++i){
    digitalWrite(FAULT, 1);
    delay(500);
    digitalWrite(FAULT, 0);
    delay(500);
  }
  delay(500);
  digitalWrite(FAULT, 1);
}

/// check various bits of state and maybe go into failure mode
/// @return true on entering failure-mode
bool checkFailure()
{
  // already failed.  Can't fail again!
  if(currentMode == MODE_FAILED){
    return false;
  }
  
  if(failflag){
    indicateFailure(CODE_SENSOR);
    return true;
  }
  else if(checkfan && fanrpm < rpm_stall){
    indicateFailure(CODE_FANSTALL);
    return true;
  }
  else if(temperature >= over_temp){
    indicateFailure(CODE_OVERHEAT);
    return true;
  }

  return false;
}


/// polling loop, variable rate but mostly 5Hz for the RPM PID loop
void loop()
{
  unsigned long now=millis();

  // poll-blink.
  digitalWrite(FAULT, currentMode != MODE_OFF);

  if(now - measurementTime > Ts_rpm){
    long window[periodWindowLength];

    // observe speed measurements; copy shared values with interrupts off
    noInterrupts();
    long wPtr=periodWindowPtr;
    for(char k=0;k<periodWindowLength;++k)
      window[k]=periodWindow[k];    
    interrupts();

    long pulseCount=wPtr-prevPeriodPtr;
    prevPeriodPtr=wPtr;

    // filter the speed measurements to get rid of spurious
    char goodCount;
    float meanPeriod=filterPeriod(&window[0], &goodCount, wPtr, min(pulseCount, periodWindowLength));
    fanrpm=(pulseCount && goodCount) ? timer_rpm/meanPeriod : 0;

    // look at temp sensors
    tempMeasure();

    // run control loops
    temppid.Compute();
    rpmpid.Compute();

    // set the fan power
    OCR2A=(int) (255-fanpwm);

    measurementTime=now;
  }

  // end blink
  if(currentMode != MODE_FAILED){
    digitalWrite(FAULT, currentMode == MODE_OFF);
  }

  // look for (new) major issues
  if(!checkFailure()){
    
    // run the mode-specific poll function
    (*pollfuncs[currentMode])();
  }

  delay(nextPoll);
}

/// step the state-machine.
void changeMode(int mode)
{
  if(mode >= 0 && mode < MODE_COUNT){ 
    modeChangeTime=millis();
    (*enterfuncs[mode])();
    currentMode=mode;
  }  
}

/// initialisation mode, fan on full-power to check that it works
void enter_init()
{
  digitalWrite(MUTE, 1);
  digitalWrite(RELAY, 0);
  digitalWrite(FANPWR, 1);
  checkfan=false;     // only just starting up
  fanpwm=255;
  OCR2A=(int) (255-fanpwm);

  nextPoll=Ts_rpm;
}

/// end of init-mode, make sure fan is running and everything is good.
void poll_init()
{
  unsigned long dt=millis() - modeChangeTime;

  // should have spun-up by now, so enable RPM-checking.
  if(dt > init_delay - 2*nextPoll){
    checkfan=true;
  }
  
  // move to off-state
  if(dt > init_delay){
    changeMode(MODE_OFF);
  }
}

/// enter the off-state; mute, power-down, no fan.
void enter_off()
{
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 0);
  checkfan=false;
  nextPoll=20;
}

/// in off-state, check to ask if there's a remote-signal for on
void poll_off()
{
  // turn back on?
  if(!digitalRead(REMOTE))
    changeMode(MODE_DELAY);
}

/// enter cooldown: mute, power-down, fan still on
void enter_cooldown()
{
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 1);
  checkfan=true;
  
  // don't work too hard...
  tempSetpoint=high_temp;

  nextPoll=Ts_rpm;
}

/// check in cooldown, look for low-temp or remote-signal for on
void poll_cooldown()
{
  // power is back!
  if(!digitalRead(REMOTE)){
    changeMode(MODE_DELAY);
  }
  // wait until cool
  else if(temperature < high_temp){
    changeMode(MODE_OFF);    
  }
}

/// delay mode: bring up the fan and mute, but no power yet.
/// setting different delays on each amp gives power sequencing
void enter_delay()
{
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 1);
  checkfan=false;
  // come up in slow mode
  tempSetpoint=high_temp;
  nextPoll=Ts_rpm;
}

/// check at end of delay: power-up or fail
void poll_delay()
{
  if(digitalRead(REMOTE)){
    // user changes mind / glitch on REMOTE line.
    changeMode(MODE_COOLDOWN);
    return;
  }
  
  // after finite time, we power-up unless overheated
  if(millis() - modeChangeTime > on_delay){
    changeMode(MODE_MUTE);
  }
}

/// mute-mode: amplifier and fan on, still in mute for a bit.
void enter_mute()
{
  digitalWrite(RELAY, 1);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 1);
  checkfan=true;    // fan should be going by now (spun up during MODE_DELAY)
  nextPoll=Ts_rpm;  
}

/// wait for end of mute-time, check that fan is running
void poll_mute()
{
  // after finite time, we come out of mute
  if(millis() - modeChangeTime > mute_delay){    
    if(digitalRead(REMOTE)){
      // user changed mind, turning off
      changeMode(MODE_COOLDOWN);
    }
    else{
      // go!
      changeMode(MODE_RUN);
    }
  }
}

/// begin run-mode: this is the primary, everything-on, do-stuff mode.
void enter_run()
{
  digitalWrite(RELAY, 1);
  digitalWrite(MUTE, 0);
  digitalWrite(FANPWR, 1);
  checkfan=true;
  tempSetpoint=high_temp; 
  nextPoll=Ts_rpm;
}

/// inside the run-mode; look for conditions that require a mode-change.
void poll_run()
{
  // remote says stop
  if(digitalRead(REMOTE)){
    changeMode(MODE_COOLDOWN);
    return;
  }

  if(tempSetpoint == high_temp && fanrpm > threshold_up){
    /// amp under load, speed up the fan
    tempSetpoint = low_temp;    
  }    
  else if(tempSetpoint == low_temp && fanrpm < threshold_down){
    // amp unloaded and cooled, slow down
    tempSetpoint = high_temp;
  }
}

/// failure: terminal mode where we shut power off and leave the fan on until cool or siezed
void enter_failed()
{
  // shut it down
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 1);
  digitalWrite(FAULT, 1);
 
  // run fan flat out, ignore temperature
  temppid.SetMode(MANUAL);
  rpmSetpoint=rpm_max;
  OCR2A=(int) 0;
  
  nextPoll=Ts_rpm;
}

/// checks in fail-mode: turn fan off once cool.
void poll_failed()
{
  // transition to OFF mode once cooled, sensors OK and remote de-asserted.
  if(temperature < high_temp && !failflag && digitalRead(REMOTE)){
    changeMode(MODE_OFF);
    return;
  }

  if(temperature < high_temp || fanrpm < rpm_stall){
    digitalWrite(FANPWR, 0);
    nextPoll=1000;
  }
}

