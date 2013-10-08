/*
	Raspberry Pi VeeOp - The Vehicle Operation Platform
	---------------------------------------------------
	Detects the ignition state of the vehicle, and powers on and off the Raspberry Pi -- with it's consent.
	Allowing the Pi to turn off gracefully, saving the integrity of it's SD card, and keeping the raspberry pi on when you want it on.
	----------------------------------------------------
	Communicates with the Raspberry Pi as an i2c slave to it, listening to commands on this custom API.
	
*/


// ------------------------------------------ -
// -- Pin Definitions!                   --- -
// ---------------------------------------- -

#define PIN_RASPI_RELAY 3
#define PIN_IGNITION 2
#define PIN_DEBUG_LED 13

// ------------------------------------------ -
// -- Command Buffer & Command variables --- -
// ---------------------------------------- -
// -- How's a command sent?
// Firstly, it's 4 bytes, first byte is command, second and third are parameters, and fourth is 0x0A (end-of-line/new-line)
// 1st Byte: The Command (has to be NON-0x0A)
// 2nd Byte: First byte in parameters.
// 3rd Byte: Second byte in parameters.
// 4th Byte: 0x0A, the end of the command.

// What's the maximum index for the param buffer? (We count the command, plus two parameters, which is 3. Excludes the end of a command.)
#define MAX_COMMAND_PARAMETERS 3
#define END_OF_COMMAND 10

// ------------------------------------------ -
// -- Command definitions ------------------ -
// ---------------------------------------- -
// These are the possible commands 
// you can issue. (never use 10! aka 0x0A, that's our end-of-command byte.)
// (that's why it starts at 11 --> "turn it up to 11")

#define CMD_GET_IGNITION_STATE 11
#define CMD_GET_LAST_IGNITION_CHANGE_SECONDS 12
#define CMD_GET_LAST_IGNITION_CHANGE_MINUTES 13
#define CMD_ECHO 14
#define CMD_PAT_WATCHDOG 15

#define CMD_DEBUG_SET_IGN_DETECT 100
#define CMD_DEBUG_SET_IGN_STATE 101
#define CMD_DEBUG_GET_IGN_DETECT 102
#define CMD_DEBUG_GET_TEST_VALUE 103
#define CMD_DEBUG_GET_WDT_STATE 104

// ------------------------------------------ -
// -- Error Definitions -------------------- -
// ---------------------------------------- -
// Errors, they happen. 
// We define the possibilities here.

#define ERR_BUFFER_OVERFLOW 1
#define ERR_COMMAND_UNKNOWN 2
#define ERR_COMMAND_INCOMPLETE 3


// ----------------------------------------- -
// -- Ignition Debounce Definition -------- -
// --------------------------------------- -
// used in debounceIgnition()
// defines the retry interval, and sequential successes to consider a digital pin change

#define CHECK_IGNITION_INTERVAL 50 				// We check for the ignition this many millis.
#define CHECK_IGNITION_RETRIES 3 				// How many times in a row does the ignition have to match?

// ----------------------------------------- -
// -- WdT State Definitions --------------- -
// --------------------------------------- -

#define WATCHDOG_STATE_WATCHING 0
#define WATCHDOG_STATE_SHUTDOWN 1
#define WATCHDOG_STATE_BOOTING 2
#define WATCHDOG_STATE_IDLE 3

#define USE_DEBUG_MODE false

// --------------------------------------------------------------------------
// vOP::vOP : The constructor.

#include "vOP.h"
#include "Arduino.h"
#include <Wire.h>


vOP::vOP() {

	// -- Over-arching variables. ------------------------------------------------------
	// Our i2c address.
	i2c_address = 4;

	// Turn this on for extra serial debug info.
	debug_mode = USE_DEBUG_MODE;

	// -- Debug Variables --------------------------------------------------------------
	
	debug_ign_debounce = 1;			// Turns off ignition detection, only useful for debugging.
	test = 1;						// I keep a test variable around for development.

	// -- Error variables --------------------------------------------------------------
	error_flag = 1; 	// Denotes an error


	// -- Command and buffer variables -------------------------------------------------
	command = 1;			// The issued command.
	param_buffer[2];		// The two posible bytes for the command parameters.
	command_complete = 1;	// Did we finish getting the command?

	// -- Stateful Device Information ---------------------------------------------------
	ignition_state = false; 			// 0 = off, 1 = on.
	ignition_delta_time = 0;			// The time when the ignition was last changed.
	raspberry_power = false;			// State of Raspberry Pi Power (0 = off, 1 = on)

	// ----------------------------------------
	// -- Watchdog Timer (WdT) Variables ------
	// ----------------------------------------
	// In watchdog mode, this micro waits for the raspi to stop sending watchdog pats, and then shuts it down.
	// In the positive case, when the ignition is off, it won't power it up until the ignition comes back on.
	// In the negative case, the ignition is still on, but no WdT pat is received -- it will just turn it off for a moment, and then back on.
	// I chose the term pat, as opposed to kick. It's just more polite: http://en.wikipedia.org/wiki/Watchdog_timer#Watchdog_restart

	watchdog_state = WATCHDOG_STATE_IDLE;		// This is the current state of the watchdog.

	watchdog_mode = true;						// When not in watchdog mode, turns off by request only.
	watchdog_shutdown_initiated = false;		// Are we going to shutdown? If we're in this mode, we're waiting to shutdown (interruptible by a pat)

	watchdog_last_pat = 0;						// When's the last time they pet the dog?
	watchdog_timeout_interval = 20;				// How long can we wait between pats? (SECONDS) If we don't see a pat in this long, we begin to shutdown power.

	watchdog_turnoff_interval = 30; 			// How long after the watchdog fails to turn it off?
	watchdog_turnoff_time = 0;					// And the next time we turn off (set when it fails.)

	watchdog_next_run = 0;						// When's the next time the watchdog will run?
	watchdog_run_interval = 5;					// And this is how often it runs. (SECONDS)

	watchdog_boot_time = 0;						// When's the time we mark a boot initiated?
	watchdog_boot_interval = 60;				// How long do we give the raspberry pi to boot? (SECONDS)

	// ------------------------------------------ -
	// -- Power Timer Variables ---------------- -
	// ---------------------------------------- -

	power_minimum_off_interval = 5;	// Minimum number of seconds the pi can be off (in order to reboot) (SECONDS)
	power_minimum_off_time = 0;		// The time we turned it off.

	// ------------ and from the original setup

	// initialize the pin to turn the relay on, as an output.
	pinMode(PIN_RASPI_RELAY, OUTPUT);
	digitalWrite(PIN_RASPI_RELAY, HIGH);


	// Here's our debug LED, it's an output.
	pinMode(PIN_DEBUG_LED, OUTPUT);

	// listen on the ignition, as input
	pinMode(PIN_IGNITION, INPUT);

	// Initialize i2c, give it the address, and the methods to call on it's events.

	/*
	Wire.begin(i2c_address);	
	Wire.onReceive(receiveData_wrapper);
	Wire.onRequest(fillRequest_wrapper);
	*/


	if (debug_mode) {
	  Serial.begin(9600);
	  debugIt("Application started.");
	}

	// Trying an init on the error flag.
	error_flag = 0;
	// and the ignition state.
	// ignition_state = true;


	// set the test to 0.
	// test = 0;

	// ignition_delta_time = millis();

}

// --------------------------------------------------------------------------
// -- bootUpHandler: Turns on the raspberry pi when necessary.

void vOP::bootUpHandler() {

	// If the raspberry pi is off...
	if (!raspberry_power) {
		// And the ignition is on...
		if (ignition_state) {
			// If we've been off for long enough (in the case of a reboot scenario, this is important.)
			if ((unsigned long)(millis() - power_minimum_off_time) >= (power_minimum_off_interval*1000)) {
				// Then we need to turn the raspberry pi on!
				debugIt("Turning raspberry pi on!");
				// Set the pin state, and turn on the relay.
				digitalWrite(PIN_RASPI_RELAY, LOW);
				// And save it in our stateful variable.
				raspberry_power = true;
				// Now we tell the watchdog we're in a booting state.
				watchdog_state = WATCHDOG_STATE_BOOTING;
				// And we give it a grace period.
				watchdog_boot_time = millis();
			}
		}
	}

}

void vOP::shutDownHandler() {

	debugIt("Shutting down raspberry pi.");
	// Turn the raspberry pi off, at the relay.
	digitalWrite(PIN_RASPI_RELAY, HIGH);
	// Note when we turned it off (in case we're rebooting, so we can have it off for a set period)
	power_minimum_off_time = millis();
	// And we note that we've turned it off in our stateful variables.
	raspberry_power = false;

}

// --------------------------------------------------------------------------
// -- watchDog: Shutdown Raspberry Pi based on watch dog pats.
// Who watches the watcher?

void vOP::watchDog() {

	// Only when watchdog mode is active.
	if (watchdog_mode) {

		// Let's only check this on an interval.
		if ((unsigned long)(millis() - watchdog_next_run) >= (watchdog_run_interval*1000)) {

			debugIt("checkin state.");
			debugItDEC(watchdog_state);
		
			// Depending on the state of the watchdog timer, we behave differently.
			switch(watchdog_state) {

				case WATCHDOG_STATE_WATCHING:
					// So now, we see if we've missed a watchdog pat.
					if ((unsigned long)(millis() - watchdog_last_pat) >= (watchdog_timeout_interval*1000)) {
						// That looks like a missed watchdog pat.
						debugIt("Watch dog pats failed, moving into shutdown mode.");
						// Now that we're missing watchdog timers. We need to know how long until we're going to shut 'er down.
						// So we'll cascade another timer here, the shutdown timer.
						test++;
						watchdog_state = WATCHDOG_STATE_SHUTDOWN;
						// Set the time that timer will run, now.
						watchdog_turnoff_time = millis(); // + (watchdog_turnoff_interval*1000);
					}
					break;

				case WATCHDOG_STATE_SHUTDOWN:
					if ((unsigned long)(millis() - watchdog_turnoff_time) >= (watchdog_turnoff_interval*1000)) {
						test++;
						// It's time to shut 'er down.
						// So first we issue a shutdown, and set the watchdog state to be idle.
						debugIt("Issuing shutdown due to watchdog pats.");
						shutDownHandler();
						watchdog_state = WATCHDOG_STATE_IDLE;
					}
					break;

				case WATCHDOG_STATE_BOOTING:
					// If the watchdog is booting.... we just stick around here.
					// Waiting for a pat. When the pat is received, the watchdog is reset, and we're put into the "watching" state.
					// But, eventually we have to timeout, and reset this mother.
					if ((unsigned long)(millis() - watchdog_boot_time) >= (watchdog_boot_interval*1000)) {
						// If we hit this, we haven't gotten a pat in the allowed boot time.
						debugIt("Boot failed, no watch dog pats before allowed time, reboot starting (if ignition up)");
						// So we issue a shutdown.
						shutDownHandler();
						// And we go idle.
						watchdog_state = WATCHDOG_STATE_IDLE;
					}
					break;

				case WATCHDOG_STATE_IDLE:
					// We don't actually do anything, we... sit idle.
					break;

			}

			// And set the next time we'll look for this.
			watchdog_next_run += watchdog_run_interval*1000;

		}
		
	}

}

void vOP::resetWatchDog() {

	// Set the time we expect the next pat.
	watchdog_last_pat = millis();
	// And since the watchdog has been pat, we also reset the watchdog state (so that we either enable it now [in the case of booting], or cancel a shutdown [in the case of, yep, a shutdown])
	watchdog_state = WATCHDOG_STATE_WATCHING;

}

// --------------------------------------------------------------------------
// -- fillRequest: What happens when there's a request from the i2c master.
// Which really means, handling the command that was read in receiveData()

void vOP::fillRequest() {

	// We return result data depending on the command. 
	// Sometimes we result in an int, we default this as true. If not, we set the bytes directly.
	bool use_int = true;
	unsigned int result_data = 0;

	// Here's the two bytes we return (we pack the int in these if true above, otherwise, set them yourself.)
	byte return_buffer[2];

	
	if (command_complete == 1) {
		// -- Command handler.
		if (error_flag == 0) {

			// No error at this point.
			switch (command) {
				case CMD_GET_IGNITION_STATE:
					// Simple, send them the latched ignition state.
					result_data = ignition_state;
					break;

				case CMD_GET_LAST_IGNITION_CHANGE_SECONDS:
					// This is simple too, we just want how long ago we changed the ignition.
					// Get it in seconds here.
					result_data = ignitionChangedLast(true);
					break;

				case CMD_GET_LAST_IGNITION_CHANGE_MINUTES:
					// And we send false to get minutes here.
					result_data = ignitionChangedLast(false);
					break;

				case CMD_ECHO:
					// Simply echo back the bytes that were send in the parameters.
					use_int = false;
					return_buffer[0] = param_buffer[0];
					return_buffer[1] = param_buffer[1];
					break;

				case CMD_PAT_WATCHDOG:
					// We just pat the dog, let's set his next runtime.
					resetWatchDog();
					break;

				// --------------------- debug_mode METHODS

					// Set the ignition detect according to the first param
					case CMD_DEBUG_SET_IGN_DETECT:
						debug_ign_debounce = param_buffer[0];
						break;

					// Set the ignition detect according to the first param
					case CMD_DEBUG_SET_IGN_STATE:
						if (ignition_state != param_buffer[0]) {
							ignition_state = param_buffer[0];
							ignition_delta_time = millis();
						}
						break;

					// Set the ignition detect according to the first param
					case CMD_DEBUG_GET_IGN_DETECT:
						result_data = debug_ign_debounce;
						break;

					// Get the test value, usefully for debugging discrete values.
					case CMD_DEBUG_GET_TEST_VALUE:
						result_data = test;
						break;

					case CMD_DEBUG_GET_WDT_STATE:
						result_data = watchdog_state;
						break;
					
				// --------------------- end debug_mode METHODS

				default:
					// Command is unknown.
					result_data = 0;
					error_flag = ERR_COMMAND_UNKNOWN;
					break;
			}

		} else {
			// Ok, there's an error here. We won't try to handle the command.
			// Let's set the result_data to 0.
			result_data = 0;
		}
	} else {

		// We never completely got that command.
		// Chances are you'll see the byte that's wrong as the "command" byte in the return. 
		error_flag = ERR_COMMAND_INCOMPLETE;

	}

	// Go ahead and convert that integer result_data down into a byte array. (if we're saying we're using an int)
	// http://stackoverflow.com/questions/3784263/converting-an-int-into-a-4-byte-char-array-c
	if (use_int) {
		return_buffer[0] = (result_data >> 8) & 0xFF;
		return_buffer[1] = result_data & 0xFF;
	}

	// Gather together the instructions to send...
	byte writer[] = {error_flag,command,return_buffer[0],return_buffer[1]};

	// And send it over the wire!	
	Wire.write(writer,4);

	// Now we have to reset errors, otherwise, we can get stuck.
	error_flag = 0;

}

// --------------------------------------------------------------------------------
// -- receiveData : Event to handle incoming data, e.g. commands from the master.
// This really ammounts to a parser for incoming data.
// The master will write us a 3-byte array, followed by an new-line (0x0A) character --> the master will make a request after sending 0x0A
// The first byte is the command, the second two bytes are parameters (and the last is end-of-line)

void vOP::receiveData(int byteCount){

	byte buffer_index = 0;	// The Index for writing to the buffer

	while(Wire.available()) {

		// Let's get that byte.
		byte inbyte = Wire.read();
		
		// Always assume command is incomplete, mark complete only on receipt of end of command
		command_complete = 0;
		
		// debugIt("over set");
		// debugIt(command_complete);
		
		// What index are we reading?
		switch (buffer_index) {

			case 0:
				// Ok, this is the first index. If it's end-of-line, it's the end of the command.
				if (inbyte != END_OF_COMMAND) {
					// It's a command, store that.
					command = inbyte;
				} else {
					// That's good, it's the end of the command.
					// Let's note that we completely got the command.
					command_complete = 1;
					
					// debugIt("inner set");
					// debugIt(command_complete);
					
				}
				break;

			default:
				// If the buffer is not yet full, we're going to populate it.
				if (buffer_index < MAX_COMMAND_PARAMETERS) {

					// Place a byte into the buffer with each read, and increment the index at which it is placed.
					// We subtract one to account for the command at position 0.
					param_buffer[buffer_index-1] = inbyte;
					
				} else {
					// Not bueno. That's a buffer overflow.
					error_flag = ERR_BUFFER_OVERFLOW;
				}
				break;

		}

		// We're done processing that byte, increment in the parameter index.
		buffer_index++;

	}

	// No more bytes available, we'll reset the buffer index (redundant)
	buffer_index = 0;

}


// the heart of the matter, the loop routine.

void vOP::loop() {
	// !bang, you'll want to turn this back, nicely!
	// digitalWrite(PIN_RASPI_RELAY, HIGH);   // turn the LED on (HIGH is the voltage level)

	// ------------------------------------- Test LED flashing
	/*
	digitalWrite(PIN_DEBUG_LED, HIGH);
	delay(750);
	digitalWrite(PIN_DEBUG_LED, LOW);
	delay(750);
	*/

	// Let's run our ignition debounce routine.
	if (debug_ign_debounce) {
		debounceIgnition();
	}

	// Fire off the watchdog. (Method knows if it's active or not.)
	watchDog();

	// Turn on the raspberry pi if application
	bootUpHandler();


	// boolean ignition = digitalRead(PIN_IGNITION);

}

// --------------------------------------------------------------------------------
// -- debounceIgnition : Gracefully latch the state of the ignition.

void vOP::debounceIgnition() {

	static unsigned long debounce_next_ignition_time = 0; 	// The last time we checked the ignition.
	static byte debounce_last_ignition_state = 0;			// Our last ignition state
	static byte debounce_counter_ignition = 0;				// How many times for the same ignition?

	// --
	// NOTE: Yo. This pin comes down slowly.
	// --

	// Rollover example from: http://www.baldengineer.com/blog/2012/07/16/arduino-how-do-you-reset-millis/
	// if ((unsigned long)(millis() - waitUntil) >= interval)

	// Is it time for a check?
	if ((unsigned long)(millis() - debounce_next_ignition_time) >= CHECK_IGNITION_INTERVAL) {

		// Read it's state.
		bool now_ignition = digitalRead(PIN_IGNITION);
		
		// Is that the same as our last read?
		if (now_ignition == debounce_last_ignition_state) {

			// That's good, we got the same value in a row -- we want to increment our same counter.
			debounce_counter_ignition++;
			
			if (debounce_counter_ignition >= CHECK_IGNITION_RETRIES) {

				// Reset the counter.
				debounce_counter_ignition = 0;

				// Did it change from the latched state? We will latch the new value if so.
				if (ignition_state != now_ignition) {

					// Latched it.
					ignition_state = now_ignition;
					// Now let's store what time we did this.
					ignition_delta_time = millis();

				}
			}

		} else {
			// Looks like it's flapping.
			// We need to reset our count.
			debounce_counter_ignition = 0;
		}

		// Keep that last state.
		debounce_last_ignition_state = now_ignition;
		// And the next time we check.
		debounce_next_ignition_time += CHECK_IGNITION_INTERVAL;

	}

}

// --------------------------------------------------------------------------------
// -- ignitionChangedLast : When did we change that?
// if "seconds" is true, then returns seconds.
// else, if false, returns minutes.

unsigned int vOP::ignitionChangedLast(bool seconds) {
	
	long now = millis();
	long last = ignition_delta_time;
	long delta =  now - last;
	delta = delta / 1000;
	if (!seconds) {
		delta = delta / 60;
	}
	
	return (unsigned int)delta;

}


// the infamous setup routine.

// --------------------------------------------------------------------------------
// -- debugIt : Print a serial line if the debug parameter is set on.

void vOP::debugIt(char *msg) {

	if (debug_mode) {
		Serial.println(msg);
	}

}

void vOP::debugItDEC(byte msg) {

	if (debug_mode) {
		Serial.println(msg,DEC);
	}

}

// ------------------------------- reference consider

// --------------------------------------------------------------------------
// -- WRAPPERS: MC Hammer & Doug E. Fresh
// So, you can't use a call-back that's a member function.
// So you're gonna have to wrap it up with a pointer.
// Fairly good stack exchange answer led me to it: http://stackoverflow.com/questions/9027456/no-matching-function-error-when-using-attachinterrupt
// And there's even more detail should you decide to induldge @ http://www.newty.de/fpt/callback.html#static

/*
void* point2receive;

void vOP::receiveData_wrapper(int numBytes){
   // explicitly cast to a pointer to Classname
   vOP* mySelf = (vOP*) point2receive;

   // call member
   mySelf->receiveData(numBytes);
}


void* point2fill;

void vOP::fillRequest_wrapper(){
   // explicitly cast to a pointer to Classname
   vOP* mySelf = (vOP*) point2fill;

   // call member
   mySelf->fillRequest();
}
*/
