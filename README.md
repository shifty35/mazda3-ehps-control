# mazda3-ehps-control
Arduino Nano Control of Mazda 3 EHPS Pump via CANBUS

There are two variants of the Mazda 3 EHPS available, the early version, and the late version.  The early version has three different connectors, and the late version only has two.  The third connector on the early version is for a steering angle sensor.  In the late version, the steering angle data is provided to the pump over the CAN network by the BCM, as stability control was an option on these vehicles.  This code will work for either pump by emulating the steering angle sensor with two digital outputs, as well as sending the steering angle data to the pump over CAN.

The protocol is fairly simple: The pump expects a message on 0x201 that contains both the engine speed and vehicle speed.  The pump will not run until it receives a message indicating that the engine is above 500 rpm or so.  Increasing vehicle speed will gradually reduce assist.

The late model pump recieves steering angle changes via CAN on 0x082.  This message is not an absolute steering angle, rather a delta value.  The early model expects steering angle changes via the steering angle sensor inputs.

The "idle" pump speed is fairly low and will yield poor P/S performance without the steering angle input.  If you do not have a power steering angle input, consider using a potentiometer to dial in a "constant" level of steering change to suit.

This code is oriented around my use case, which is a 2006 Mazda MX5.  One MCP2515 listens on the vehicle network, and one is on a separate CAN network with only the pump.  This is because the MX5 already has a message being broadcast on 0x201 in an incorrect format.  This example listens for the MX5 broadcast, translates the RPM, vehicle speed, and steering angle values to the correct formats, and broadcasts back to the pump.

Similar adaptations should be able to be made for basically any vehicle with a CAN network - find the address and format of the vehicle speed, RPM, and steering inputs, and convert to the format expected by the Mazda 3 pump.

Needed Items:

Arduino Nano or compatible
MCP2515 or compatible


