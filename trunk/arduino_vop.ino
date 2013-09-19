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
// you can issue.

#define CMD_GET_IGNITION_STATE 11
#define CMD_GET_LAST_IGNITION_CHANGE 12

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
// -- Result Buffer & Result varaibles ----
// ----------------------------------------



// --------------------------------------------------------------------------
// -- fillRequest: What happens when there's a request from the i2c master.
// Which really means, handling the command that was read in receiveData()

void fillRequest() {

	Serial.println("Filling request."); 

	// Let's test with an int.
	unsigned int result = 0;
	
	if (command_complete == 1) {
		// -- Command handler.
		if (error_flag == 0) {
			
			// No error at this point.
			switch (command) {
				case CMD_GET_IGNITION_STATE:
					result = 2600;
					break;

				case CMD_GET_LAST_IGNITION_CHANGE:
					result = 420;
					break;

				default:
					// Command is unknown.
					result = 0;
					error_flag = ERR_COMMAND_UNKNOWN;
					break;
			}

		} else {
			// Ok, there's an error here. We won't try to handle the command.
			// Let's set the result to 0.
			result = 0;
		}
	} else {

		// We never completely got that command.
		// Chances are you'll see the byte that's wrong as the "command" byte in the return. 
		error_flag = ERR_COMMAND_INCOMPLETE;

	}

	// Let's just echo back the command and params now.
	// byte writer[] = {command,param_buffer[0],param_buffer[1]};

	// Go ahead and convert that integer result down into a byte array.
	// http://stackoverflow.com/questions/3784263/converting-an-int-into-a-4-byte-char-array-c
	byte intbuffer[2];
	intbuffer[0] = (result >> 8) & 0xFF;
	intbuffer[1] = result & 0xFF;

	byte writer[] = {error_flag,command,intbuffer[0],intbuffer[1]};


	Wire.write(writer,4);

	// Now we have to reset errors, otherwise, we can get stuck.
	error_flag = 0;

}


// --------------------------------------------------------------------------------
// -- receiveData : Event to handle incoming data, e.g. commands from the master.
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
	delay(2000);
	//digitalWrite(PIN_RASPI_TRANSISTOR, HIGH);   // turn the LED on (HIGH is the voltage level)
	digitalWrite(PIN_RASPI_RELAY, HIGH);   // turn the LED on (HIGH is the voltage level)
	while(1) {

		// ------------------------------------- Test LED flashing
		digitalWrite(PIN_DEBUG_LED, HIGH);
		delay(750);
		digitalWrite(PIN_DEBUG_LED, LOW);
		delay(750);

		// Let's show the value of the ignition
		// NOTE: Yo. This pin comes down slowly.
		boolean ignition = digitalRead(PIN_IGNITION);

		if (DEBUG) {
			// Serial.println("Value of ignition"); 
			// Serial.println(ignition); 
		}
    
  	}

}

// the infamous setup routine.
void setup() {

	// initialize the pin to turn the relay on, as an output.
	pinMode(PIN_RASPI_RELAY, OUTPUT);

	// Here's our debug LED, it's an output.
	pinMode(PIN_DEBUG_LED, OUTPUT);

	// listen on the ignition, as input
	pinMode(PIN_IGNITION, INPUT);

	if (DEBUG) {
	  Serial.begin(9600);
	}

	// Initialize i2c, give it the address, and the methods to call on it's events.
	Wire.begin(I2C_ADDRESS);	
	Wire.onReceive(receiveData);
	Wire.onRequest(fillRequest);

	// Trying an init on the error flag.
	error_flag = 0;
  
}
