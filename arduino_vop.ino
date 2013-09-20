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
#define PIN_RASPI_TRANSISTOR_REROUTE 7
#define PIN_DEBUG_LED 13

byte test = 1; 

// ----------------------------------------
// -- Command Buffer & Command variables --
// ----------------------------------------
// -- How's a command sent?
// Firstly, it's 4 bytes, first by is command, second and third are parameters, and fourth is 0x0A (end-of-line/new-line)
// 1st Byte: The Command (has to be NON-0xA)
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

#define CMD_GET_IGNITION_STATE 11
#define CMD_GET_LAST_IGNITION_CHANGE_SECONDS 12
#define CMD_GET_LAST_IGNITION_CHANGE_MINUTES 13
#define CMD_ECHO 14

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

byte ignition_state = 1; 				// 0 = off, 1 = on.
unsigned long ignition_delta_time = 0;	// The time when the ignition was last changed.

// ----------------------------------------
// -- Result Buffer & Result varaibles ----
// ----------------------------------------



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

	byte writer[] = {error_flag,command,return_buffer[0],return_buffer[1]};
	
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
	debounceIgnition();
	// boolean ignition = digitalRead(PIN_IGNITION);

}

// --------------------------------------------------------------------------------
// -- debounceIgnition : Gracefully latch the state of the ignition.


// quasi-localized variables for debouncing.
#define CHECK_IGNITION_MILLIS 50 				// We check for the ignition this many millis.
#define CHECK_IGNITION_RETRIES 3 				// How many times in a row does the ignition have to match?
unsigned long debounce_last_ignition_time = 0; 	// The last time we checked the ignition.
byte debounce_last_ignition_state = 0;			// Our last ignition state
byte debounce_counter_ignition = 0;				// How many times for the same ignition?

void debounceIgnition() {

	// --
	// NOTE: Yo. This pin comes down slowly.
	// --

	// Check what time it is....
	unsigned long now = millis();

	// Is it time for a check?
	if ((debounce_last_ignition_time + CHECK_IGNITION_MILLIS) <= now) {

		// Read it's state.
		byte now_ignition = byte(digitalRead(PIN_IGNITION));
		
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
		// And the last time we checked.
		debounce_last_ignition_time = now;

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
	ignition_state = 1;
	// set the test to 0.
	test = 0;

	// ignition_delta_time = millis();
  
}
