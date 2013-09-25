/*
	Raspberry Pi VeeOp - The Vehicle Operation Platform
	---------------------------------------------------
	Detects the ignition state of the vehicle, and powers on and off the Raspberry Pi -- with it's consent.
	Allowing the Pi to turn off gracefully, saving the integrity of it's SD card, and keeping the raspberry pi on when you want it on.
	----------------------------------------------------
	Communicates with the Raspberry Pi as an i2c slave to it, listening to commands on this custom API.
	
*/

// Use wire, i2c is rather important here. 
#include <Wire.h>
// Our i2c address.
#define I2C_ADDRESS 4

// Turn this on for extra serial debug info.
#define DEBUG false

// ----------------------------------------
// -- Pin Definitions!                   --
// ----------------------------------------

#define PIN_RASPI_RELAY 3
#define PIN_IGNITION 2
#define PIN_DEBUG_LED 13

byte test = 1; 

// ----------------------------------------
// -- Command Buffer & Command variables --
// ----------------------------------------
// -- How's a command sent?
// Firstly, it's 4 bytes, first byte is command, second and third are parameters, and fourth is 0x0A (end-of-line/new-line)
// 1st Byte: The Command (has to be NON-0x0A)
// 2nd Byte: First byte in parameters.
// 3rd Byte: Second byte in parameters.
// 4th Byte: 0x0A, the end of the command.

// What's the maximum index for the param buffer? (We count the command, plus two parameters, which is 3. Excludes the end of a command.)
#define MAX_COMMAND_PARAMETERS 3
#define END_OF_COMMAND 10
byte command = 1;			// The issued command.
byte param_buffer[2];		// The two posible bytes for the command parameters.
byte command_complete = 1;	// Did we finish getting the command?

// ----------------------------------------
// -- Command definitions -----------------
// ----------------------------------------
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

// ----------------------------------------
// -- Debug Variables ---------------------
// ----------------------------------------

byte debug_ign_debounce = 1;


// ----------------------------------------
// -- Error Definitions -------------------
// ----------------------------------------
// Errors, they happen. 
// We define the possibilities here.

byte error_flag = 1; 	// Denotes an error

#define ERR_BUFFER_OVERFLOW 1
#define ERR_COMMAND_UNKNOWN 2
#define ERR_COMMAND_INCOMPLETE 3


// ----------------------------------------
// -- Stateful Device Information ---------
// ----------------------------------------

bool ignition_state = false; 			// 0 = off, 1 = on.
unsigned long ignition_delta_time = 0;	// The time when the ignition was last changed.

// ----------------------------------------
// -- Ignition Debounce Definition --------
// ----------------------------------------
// used in debounceIgnition()
// defines the retry interval, and sequential successes to consider a digital pin change

#define CHECK_IGNITION_INTERVAL 50 				// We check for the ignition this many millis.
#define CHECK_IGNITION_RETRIES 3 				// How many times in a row does the ignition have to match?


// ----------------------------------------
// -- Watchdog Timer (WdT) Variables ------
// ----------------------------------------
// In watchdog mode, this micro waits for the raspi to stop sending watchdog pats, and then shuts it down.
// In the positive case, when the ignition is off, it won't power it up until the ignition comes back on.
// In the negative case, the ignition is still on, but no WdT pat is received -- it will just turn it off for a moment, and then back on.
// I chose the term pat, as opposed to kick. It's just more polite: http://en.wikipedia.org/wiki/Watchdog_timer#Watchdog_restart

bool watchdog_mode = true;						// When not in watchdog mode, turns off by request only.
bool watchdog_shutdown_initiated = false;		// Are we going to shutdown? If we're in this mode, we're waiting to shutdown (interruptible by a pat)
unsigned long watchdog_next_pat = 0;			// When's the last time they pet the dog?
unsigned int watchdog_timeout_interval = 10;	// How long can we wait between pats? (SECONDS) If we don't see a pat in this long, we begin to shutdown power.
unsigned int watchdog_turnoff_seconds = 60; 	// How long after the watchdog fails to turn it off?
unsigned long watchdog_next_run = 0;			// When's the next time the watchdog will run?
unsigned int watchdog_run_interval = 5;			// And this is how often it runs. (SECONDS)
byte watchdog_state = 0;						// This is the current state of the watchdog.

#define WATCHDOG_STATE_WATCHING 0
#define WATCHDOG_STATE_SHUTDOWN 1

// --------------------------------------------------------------------------
// -- watchDog: Shutdown Raspberry Pi based on watch dog pats.
// Who watches the watcher?

void watchDog() {

	// Only when watchdog mode is active.
	if (watchdog_mode) {

		// Let's only check this on an interval.
		if ((unsigned long)(millis() - watchdog_next_run) >= (watchdog_run_interval*1000)) {
		
			// Depending on the state of the watchdog timer, we behave differently.
			switch(watchdog_state) {

				case WATCHDOG_STATE_WATCHING:
					// So now, we see if we've missed a watchdog pat.
					if ((unsigned long)(millis() - watchdog_next_pat) >= (watchdog_timeout_interval*1000)) {
						// That looks like a missed watchdog.
						test++;
						// Now that we're missing watchdog timers. We need to know how long until we're going to shut 'er down.
						// So we'll cascade another timer here.

					}
					break;

			}

			// And set the next time we'll look for this.
			watchdog_next_run += watchdog_run_interval*1000;

		}
		
	}

}

void resetWatchDog() {

	watchdog_next_pat += millis() + watchdog_timeout_interval;

}

// --------------------------------------------------------------------------
// -- fillRequest: What happens when there's a request from the i2c master.
// Which really means, handling the command that was read in receiveData()

void fillRequest() {

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

				// --------------------- DEBUG METHODS

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
					
				// --------------------- end DEBUG METHODS

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

void receiveData(int byteCount){

	byte buffer_index = 0;	// The Index for writing to the buffer

	while(Wire.available()) {

		// Let's get that byte.
		byte inbyte = Wire.read();
		
		// Always assume command is incomplete, mark complete only on receipt of end of command
		command_complete = 0;
		if (DEBUG) {
			Serial.println("over set");
			Serial.println(command_complete);
		}

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
					
					if (DEBUG) {
						Serial.println("inner set");
						Serial.println(command_complete);
					}
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

void loop() {
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


	// boolean ignition = digitalRead(PIN_IGNITION);

}

// --------------------------------------------------------------------------------
// -- debounceIgnition : Gracefully latch the state of the ignition.

void debounceIgnition() {

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

unsigned int ignitionChangedLast(bool seconds) {
	
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
void setup() {

	// initialize the pin to turn the relay on, as an output.
	pinMode(PIN_RASPI_RELAY, OUTPUT);

	// Here's our debug LED, it's an output.
	pinMode(PIN_DEBUG_LED, OUTPUT);

	// listen on the ignition, as input
	pinMode(PIN_IGNITION, INPUT);

	// Initialize i2c, give it the address, and the methods to call on it's events.
	Wire.begin(I2C_ADDRESS);	
	Wire.onReceive(receiveData);
	Wire.onRequest(fillRequest);


	if (DEBUG) {
	  Serial.begin(9600);
	  Serial.println("Application started.");
	}


	// Trying an init on the error flag.
	error_flag = 0;
	// and the ignition state.
	ignition_state = true;
	// set the test to 0.
	// test = 0;

	// ignition_delta_time = millis();
  
}
