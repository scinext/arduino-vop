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

byte test = 0; 

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
#define PARAMETERS_MAX 3
#define END_OF_COMMAND 0x0A
unsigned char command = 0;
unsigned char param_buffer[2];		// The two posible bytes for the command parameters.
unsigned char param_index = 0;		// The Index for writing to the buffer.
bool command_error_flag = false; 	// Denotes an error while reading a command.

// ----------------------------------------
// -- Result Buffer & Result varaibles ----
// ----------------------------------------

unsigned int result = 2;


// the setup routine runs once when you press reset:
void setup() {

	// initialize the pin to turn the relay on, as an output.
	pinMode(PIN_RASPI_RELAY, OUTPUT);

	// Here's out debug LED, it's an output.
	pinMode(PIN_DEBUG_LED, OUTPUT);

	// listen on the ignition, as input
	pinMode(PIN_IGNITION, INPUT);

	if (DEBUG) {
	  Serial.begin(9600);
	}

	Wire.begin(I2C_ADDRESS);	
	Wire.onReceive(receiveData);
	Wire.onRequest(fillRequest);
  
}

// --------------------------------------------------------------------------------
// -- receiveData : Event to handle incoming data, e.g. commands from the master.
// The master will write us a 5-byte array, followed by an new-line (0x0A) character.
// The first three bytes denote the command to send, and the next two bytes are for parameters to that command.

void receiveData(int byteCount){

	while(Wire.available()) {

		// Let's get that byte.
		unsigned char in = Wire.read();
		
		// What index are we reading?
		switch (param_index) {

			case 0:
				// Ok, this is the first index. If it's end-of-line, it's the end of the command.
				if (in != END_OF_COMMAND) {
					// It's a command, store that.
					command = in;
				} else {
					// That's good, it's the end of the command, we'll keep that as a success.
					command_error_flag = false;
				}

			default:
				// If the buffer is not yet full, we're going to populate it.
				if (param_index < PARAMETERS_MAX) {

					// Place a byte into the buffer with each read, and increment the index at which it is placed.
					// We subtract one to account for the command at position 0.
					param_buffer[param_index-1] = in;

				} else {
					// Not bueno. That's a buffer overflow.
					command_error_flag = true;
				}
				break;

		}

		// We're done processing that byte, increment in the parameter index.
		param_index++;

	}

	// No more bytes available, we'll reset the buffer index.
	param_index = 0;

}

void fillRequest() {

	// Let's just echo back the command and params now.
	unsigned char writer[] = {command,param_buffer[0],param_buffer[1]};
	Wire.write(writer,3);

}


// the loop routine runs over and over again forever:
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
			Serial.println("Value of ignition"); 
			Serial.println(ignition); 
		}
    
  	}

}