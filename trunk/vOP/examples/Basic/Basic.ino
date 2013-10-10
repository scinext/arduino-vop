// --------------------------------------------------------------
//              _____ _____ 
//  _ _ ___ ___|     |  _  |
// | | | -_| -_|  |  |   __|
//  \_/|___|___|_____|__|  
//
// The Raspberry Pi Vehicle Operating Platform.
// The extensible and open source platform for
// operating your raspberry pi in a vehicle, 
// and properly powering it up and down based
// on the ignition state of your vehicle.
// --------------------------------------------------------------
// Call the library, throw in hooks to it's loop
// and it's i2c interface, and away you go!
// --------------------------------------------------------------

// Include our library.
#include <vOP.h>
// Plus the Wire library.
#include <Wire.h>


vOP vop;

void loop() {

	// Call this as frequently as you can.
	// You can call it less frequently, but, it runs rather quickly.
	vop.loop();

	// --------------------------------------------------------- 
	// -- Your code here!! :)                                 --
	// --------------------------------------------------------- 
	
}

void setup() {

	// --------------------------------------------------------- 
	// -- Your code here!! :)                                 --
	// --------------------------------------------------------- 


	// --------------------------------------------------------- 
	// -- Setup routine.                                      --
	// Some methods don't like to be run up on object         --
	// instantiation, so we run them in setup                 --
	// --------------------------------------------------------- 

	vop.setup();

	// --------------------------------------------------------- 
	// -- Wire setup.                                         --
	// Go ahead and begin on the address of your choosing.    --
	// And then call a wrapper to access the library's        --
	// methods for working with i2c.                          --
	// ---------------------------------------------------------

	Wire.begin(0x04);
	Wire.onReceive(receiveWrapper);
	Wire.onRequest(requestWrapper);



}

// --------------------------------------------------------- 
// -- The Wrappers!                                       --
// Rick The Ruler & Doug E. Fresh                         --
// Wire library is picky about calling a plain function   --
// as a callback. So, this was expedient to create this   --
// way. In the future I may try to pack this directly     --
// into the library.                                      --
// --------------------------------------------------------- 


void receiveWrapper(int numbytes) {
	vop.receiveData(numbytes);
}

void requestWrapper() {
	vop.fillRequest();
}