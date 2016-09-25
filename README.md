# fancontrol

This is a PID-based fan-speed controller for use in conjunction with an audio power
amplifier or similar linear device.  It has two temp-control modes:
 - high-temp (e.g. 45C) quiet mode
 - low-temp, high-power mode (e.g. 30C)

It defaults to high-temp mode at startup, keeping the fan as quiet as possible while
preventing the amp from getting problematically hot.  If the user cranks the amplifier power,
fan speed will be seen climbing in order to maintain temperature.  In that case, the 
temperature set-point will be brought down and the fan will run harder to keep the system
cooler while operating at a higher power level.  The purpose is to match the thermal conditions
to the load:
 - at low power, higher temp is acceptable but fan noise is not,
 - at high power, the fan is inaudible and lower temp is desirable to maximise transistor SOA.

There are two nested PID loops:
 - temperature setpoint, the output of which is fan RPM, and
 - fan speed, the output of which is a PWM signal

The inner (fan speed) loop runs at ~5Hz and the outer (temperature) loop runs at ~0.25Hz.

Amplifier power is applied only when:
 - the remote-trigger signal is asserted,
 - the fan has not stalled, 
 - the AVR is successfully getting temp readings from all sensors, and
 - all measured temperatures are below the safety-cutoff level
     - hard backup: temp sensors have an "overtemp" signal that will cut power even in the case of software failure.

There are the following modes, treated as a finite state machine:
- off (remote signal is off)
- init (full fan power at startup to overcome stiction; lasts 1s)
- cooldown (remote off, but temp still high)
- delay (remote has turned on, we are waiting before turning on the power)
- mute (amp turned on but held in mute while its power supply stabilises)
- run (normal operation)
- failed (overheat or fan dead)

When entering failure-mode, the fault-LED will flash a few times:
1: overheat
2: fan stall
3: sensor failure

Future work and missing features
 - AC detection
 - low-temp fan power-down, independent of relay
 - IR remote receiver
 - put control-parameters in EEPROM and permit changes over serial

Intended target is an Arduino Nano v3, with custom carrier/shield board.

