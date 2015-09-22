This is the code that runs on the microcontroller in a Raspberry Pi Vehicle Operating Platform (veeOp).

It's primary purpose is to provide an extensible platform upon which you run your raspberry pi in a vehicle. It makes two way communication with a Raspberry Pi to take instructions on when to properly power the raspberry pi off, and on.

As to allow the user to power down the raspberry pi properly using the ignition state of the vehicle for instruction. The user may instruct the raspberry pi to stay on for a set period after the ignition (and stay on if the ignition is again turned on during that period), or have more granular control by issuing commands to customize when/how/why the raspberry shuts down.

This code is Arduino-compatible. It runs with a companion project written in Node.js which provides a REST API for the user to interface with this microcontroller (and it's logic herein). This communication happens over I2C, where this microcontroller acts as a slave to the Raspberry Pi.