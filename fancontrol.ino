/**
 * Amplifier Fan Controller
 * (C) 2014 W Brodie-Tyrrell
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
 *  - at low power, high temp is acceptable but fan noise is not,
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
 *  - the AVR is successfully getting temp readings from both sensors, and
 *  - the heatsink temperature is below the safety-cutoff level
 *       (hard backup: relay must be wired to switch off if a TH signal is asserted)
 *
 * An LCD can be connected, which will display thermal & controller status.
 *
 * There are the following modes, treated as a finite state machine:
 * - init (full fan power at startup, lasts 1s)
 * - off (remote off)
 * - cooldown (remote off, but temp still high)
 * - delay (remote has turned on, we are waiting before turning on the power)
 * - mute (amp turned on but held in mute while stabilising)
 * - run (normal operation)
 * - failed (overheat or fan dead)
 *
 * Future work:
 *  - muting for turn-on/off thump prevention
 *  - AC detection
 *  - fan power-down, independent of relay
 *  - integrate volume-control chips and IR remote-receiver
 *  - serial link between multiple amps to synchronise volume levels
 *
 * Intended target device is an Arduino Nano, for installation in the amplifier.
 */

#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <DS1620.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <Arduino.h>

#define DS_DQ 2
#define DS_CLK 3
#define DS_RSTA 5
#define DS_RSTB 6

#define FAULT  13    // indicator LED
#define RELAY A4       // power relay to run amplifiers
#define FAN  11      // timer2 HF PWM output  (FIXED PIN)
#define FANPWR 4     // fan power-up
#define REMOTE 12    // remote-powerup input
#define SPEED 8      // ICP1=PB0=8, tacho input (FIXED PIN)
#define MUTE 7       // mute signal for de-thump
#define ACPRESENT 10  // AC detection

#define LCDD7  A0    // status display, 16x2 HD44780
#define LCDD6  A1
#define LCDD5  A2
#define LCDD4  A3
#define LCDRS  DS_CLK
#define LCDEN  A5

#define DS_CONF (DS1620::CBIT | DS1620::CPU)
#define DS_MASK 0x0F

// temp-measurement devices
DS1620 ds1620a(DS_DQ, DS_CLK, DS_RSTA);
DS1620 ds1620b(DS_DQ, DS_CLK, DS_RSTB);

// LCD display
LiquidCrystal lcd(LCDRS, LCDEN, LCDD4, LCDD5, LCDD6, LCDD7);

// control presets
const double over_temp=60.0;       // cooking, turn it all off.
const double high_temp=45.0;       // high-temp/low-speed preset
const double low_temp=40.0;        // low-temp/high-speed preset
const double threshold_up=1500;    // if fan faster than this, lower the preset
const double threshold_down=1200;   // if fan slower than this, raise the preset
const double rpm_min=450, rpm_max=2000;  // fan capabilities
const long threshold_time=60000;   // speed must cross threshold for 1 minute
const long init_delay=1000;        // fan test-spinup time at boot
const long on_delay=200;           // wait this long after remote asserted before powering up
const long mute_delay=200;         // wait this long before coming out of mute

// timer speed that we clock the tacho with
const float timer_rpm=7.5e6;
// PID sample periods
const int Ts_rpm=200, Ts_temp=4000;
// PID state-variables 
double fanrpm, fanpwm=255, rpmSetpoint=rpm_max;
double temperature=25.0, tempSetpoint=high_temp;

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

/// setup Timer2 as high-speed (32kHz) PWM
void pwmSetup()
{
  OCR2A=0xFF;    // max output
  TCCR2A=0x81;   // phase-correct PWM, non-inverted
  TCCR2B=0x01;   // no prescale, 32kHz output
}

/// main entry-point: configure the system
void setup()
{
  lcd.begin(8, 2);
  lcd.clear();

  // boot & configure the temp sensors
  if((ds1620a.getConfig() & DS_MASK) != DS_CONF)
    ds1620a.setConfig(DS_CONF);
  ds1620a.convert();
  if((ds1620b.getConfig() & DS_MASK) != DS_CONF)
    ds1620b.setConfig(DS_CONF);
  ds1620b.convert();

  // configure PIDs
  rpmpid.SetSampleTime(Ts_rpm);
  rpmpid.SetOutputLimits(0, 255);  // PWM bounds
  rpmpid.SetMode(AUTOMATIC);

  temppid.SetSampleTime(Ts_temp);
  temppid.SetOutputLimits(rpm_min, rpm_max);  // fan speed range
  temppid.SetMode(AUTOMATIC);
  tempSetpoint=high_temp;  

  // setup RPM measurement
  speedSetup();

  // setup fan PWM and other IO
  pinMode(FAN, OUTPUT);
  pinMode(FANPWR, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(FAULT, OUTPUT);
  pinMode(REMOTE, INPUT);
  pinMode(MUTE, OUTPUT);
  pinMode(ACPRESENT, INPUT);
  pwmSetup();

  digitalWrite(FAULT, 0);

  // we startup and wait for the fan to get going
  changeMode(MODE_INIT);
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

/// polling loop at about 5Hz
void loop()
{
  unsigned long now=millis();

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

    // run control loops
    temppid.Compute();
    rpmpid.Compute();

    // set the power
    OCR2A=(int) (255-fanpwm);

    temperature=max(ds1620a.getTemp(), ds1620b.getTemp())/2.0;

    measurementTime=now;
  }

  // run the mode-specific poll function
  (*pollfuncs[currentMode])();

  delay(nextPoll);
}

// display RPM and temperature on the LCD
void printRpmTemp()
{
    lcd.clear();
    lcd.print((int) fanrpm);
    lcd.setCursor(5, 0);
    lcd.print("RPM");
    lcd.setCursor(1, 1);
    lcd.print(temperature);
    lcd.print("C");  
}

void changeMode(int mode)
{
  if(mode >= 0 && mode < MODE_COUNT){ 
    modeChangeTime=millis();
    (*enterfuncs[mode])();
    currentMode=mode;
  }
}

void enter_init()
{
  digitalWrite(MUTE, 1);
  digitalWrite(RELAY, 0);
  digitalWrite(FANPWR, 1);
  fanpwm=255;
  OCR2A=(int) (255-fanpwm);

  nextPoll=Ts_rpm;
  
  lcd.clear();
  lcd.print("Fan Test");
}

void poll_init()
{
  // make sure fan successfully spun up and we're not overheated
  if(millis() - modeChangeTime > init_delay){
    if(fanrpm == 0 || temperature >= over_temp)
      changeMode(MODE_FAILED);
    else
      changeMode(MODE_OFF);
  }
}

void enter_off()
{
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 0);

  lcd.clear();
  lcd.print("Off");
  nextPoll=20;  
}

void poll_off()
{
  // turn back on?
  if(digitalRead(REMOTE))
    changeMode(MODE_DELAY);

/*    
  lcd.clear);
  lcd.print(ds1620a.getTemp(), 16);
  lcd.print(ds1620b.getTemp(), 16);
    
  lcd.setCursor(0, 1);
  lcd.print(ds1620a.getConfig(), 16);
  lcd.print(" ");
  lcd.print(ds1620b.getConfig(), 16);
*/
  ds1620a.convert();
  ds1620b.convert();
}

void enter_cooldown()
{
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 1);

  lcd.clear();
  lcd.print("Cooldown");
  nextPoll=Ts_rpm;
}

void poll_cooldown()
{
  // don't work too hard...
  tempSetpoint=high_temp;

  // power is back!
  if(digitalRead(REMOTE))
    changeMode(MODE_DELAY);
  // wait until cool
  else if(temperature < high_temp)
    changeMode(MODE_OFF);    
  // check for stall
  else if(fanrpm == 0 || temperature >= over_temp)
    changeMode(MODE_FAILED);
  else{
    lcd.clear();
    lcd.print("Cooldown");
    lcd.setCursor(1, 1);
    lcd.print(temperature);
    lcd.print("C"); 
  }
}

void enter_delay()
{
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 1);
  // come up in slow mode
  tempSetpoint=high_temp;
  nextPoll=Ts_rpm;
  
  lcd.clear();
  lcd.write("Power!");
}

void poll_delay()
{
  // after finite time, we power-up unless overheated
  if(millis() - modeChangeTime > on_delay){
    if(temperature < over_temp)
      changeMode(MODE_MUTE);
    else
      changeMode(MODE_FAILED);
  }
  
  lcd.setCursor(0, 1);
  lcd.print("cnf ");
  lcd.print(ds1620a.getConfig(), 16);
}

void enter_mute()
{
  digitalWrite(RELAY, 1);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 1);
  nextPoll=Ts_rpm;  
  
  lcd.clear();
  lcd.print("Mute");
}

void poll_mute()
{
  // after finite time, we come out of mute
  if(millis() - modeChangeTime > mute_delay)
    changeMode(MODE_RUN);

  lcd.clear();
  lcd.print(ds1620b.getConfig(), 16);

  lcd.setCursor(0, 1);
  lcd.print("tmp ");
  lcd.print((short) ds1620b.getTemp(), 16);
}

void enter_run()
{
  digitalWrite(RELAY, 1);
  digitalWrite(MUTE, 0);
  digitalWrite(FANPWR, 1);
  tempSetpoint=high_temp; 
  nextPoll=Ts_rpm;
}

void poll_run()
{
  // fan failed or overheated
  if(fanrpm == 0 || temperature >= over_temp)
    changeMode(MODE_FAILED);
  // remote says stop
  else if(!digitalRead(REMOTE))
    changeMode(MODE_COOLDOWN);
  // amp under load, speed up
  else{
    if(tempSetpoint == high_temp && fanrpm > threshold_up)
      tempSetpoint = low_temp;    
    // amp unloaded and cooled, slow down
    else if(tempSetpoint == low_temp && fanrpm > threshold_down)
      tempSetpoint = high_temp;
      
    printRpmTemp();
  }
}

void enter_failed()
{
  // shut it down
  digitalWrite(RELAY, 0);
  digitalWrite(MUTE, 1);
  digitalWrite(FANPWR, 1);
  digitalWrite(FAULT, 1);
 
  lcd.clear();
  if(fanrpm == 0)
    lcd.print("Fan Fail");
  else if(temperature >= over_temp){
    lcd.print("Overheat");
    lcd.setCursor(1, 1);
    lcd.print(temperature);
    lcd.print("C");
  }
  else
    lcd.print("Failure");
 
  // run fan flat out, ignore temperature
  temppid.SetMode(MANUAL);
  rpmSetpoint=rpm_max;
  
  nextPoll=Ts_rpm;
}

void poll_failed()
{
  // turn fan off once cooled or if stalled
  if(temperature < high_temp || fanrpm == 0){
    digitalWrite(FANPWR, 0);
    nextPoll=1000;
  }
}

