#ifndef Morse_h
#define Morse_h

#include "Arduino.h"

class vOP {
  public:
    vOP();
    void loop();
    void setup();
    void bootUpHandler();
    void shutDownHandler();
    void shutdownRequestHandler();
    void watchDog();
    void resetWatchDog();
    void fillRequest();
    void receiveData(int byteCount);
    unsigned int paramsToInt(byte a,byte b);
    void debounceIgnition();
    unsigned int ignitionChangedLast(bool seconds);
    void debugIt(char *msg);
    void debugItDEC(byte msg);
    void debugItBIN(int msg);
  private:
  	// Our i2c address.
	byte i2c_address;

	// Turn this on for extra serial debug info.
	bool debug_mode;

	// Just a test variable.
	byte test;
	byte error_flag;

	// ----------------------------------------
	// -- Command Buffer & Command variables --
	// ----------------------------------------
	// -- How's a command sent?
	// Firstly, it's 4 bytes, first byte is command, second and third are parameters, and fourth is 0x0A (end-of-line/new-line)
	// 1st Byte: The Command (has to be NON-0x0A)
	// 2nd Byte: First byte in parameters.
	// 3rd Byte: Second byte in parameters.
	// 4th Byte: 0x0A, the end of the command.
	byte command;			// The issued command.
	byte param_buffer[2];	// The two posible bytes for the command parameters.
	byte command_complete;	// Did we finish getting the command?

	// ----------------------------------------
	// -- Debug Variables ---------------------
	// ----------------------------------------

	byte debug_ign_debounce;	// Turns off ignition detection, only useful for debugging.


	// ----------------------------------------
	// -- Stateful Device Information ---------
	// ----------------------------------------

	bool ignition_state; 			// 0 = off, 1 = on.
	unsigned long ignition_delta_time;	// The time when the ignition was last changed.
	bool raspberry_power;			// State of Raspberry Pi Power (0 = off, 1 = on)


	// ----------------------------------------
	// -- Ignition Debounce Definition --------
	// ----------------------------------------
	// used in debounceIgnition()
	// defines the retry interval, and sequential successes to consider a digital pin change

	// ----------------------------------------
	// -- Shutdown Request Variables ----------
	// ----------------------------------------

	bool shutdown_request_mode;
	unsigned long shutdown_request_at;

	// ----------------------------------------
	// -- Watchdog Timer (WdT) Variables ------
	// ----------------------------------------
	// In watchdog mode, this micro waits for the raspi to stop sending watchdog pats, and then shuts it down.
	// In the positive case, when the ignition is off, it won't power it up until the ignition comes back on.
	// In the negative case, the ignition is still on, but no WdT pat is received -- it will just turn it off for a moment, and then back on.
	// I chose the term pat, as opposed to kick. It's just more polite: http://en.wikipedia.org/wiki/Watchdog_timer#Watchdog_restart

	byte watchdog_state;						// This is the current state of the watchdog.

	bool watchdog_mode;							// When not in watchdog mode, turns off by request only.
	bool watchdog_shutdown_initiated;			// Are we going to shutdown? If we're in this mode, we're waiting to shutdown (interruptible by a pat)

	unsigned long watchdog_last_pat;			// When's the last time they pet the dog?
	unsigned int watchdog_timeout_interval;		// How long can we wait between pats? (SECONDS) If we don't see a pat in this long, we begin to shutdown power.

	unsigned int watchdog_turnoff_interval; 	// How long after the watchdog fails to turn it off?
	unsigned long watchdog_turnoff_time;		// And the next time we turn off (set when it fails.)

	unsigned long watchdog_next_run;			// When's the next time the watchdog will run?
	unsigned int watchdog_run_interval;			// And this is how often it runs. (SECONDS)

	unsigned long watchdog_boot_time;			// When's the time we mark a boot initiated?
	unsigned int watchdog_boot_interval;		// How long do we give the raspberry pi to boot? (SECONDS)

	// ----------------------------------------
	// -- Power Timer Variables ---------------
	// ----------------------------------------

	unsigned int power_minimum_off_interval;	// Minimum number of seconds the pi can be off (in order to reboot) (SECONDS)
	unsigned long power_minimum_off_time;		// The time we turned it off.

};

#endif