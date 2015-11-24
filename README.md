This repository contains a static library for interfacing uBlox GPS receivers.  This interface is based on the uBlox 6 protocol specifications.

# INSTALL INSTRUCTIONS #

## Linux Ubuntu ##

### Dependencies ###

Update the package list.

		sudo apt-get update

-	CMake

		sudo apt-get install cmake
	
-	Boost

		sudo apt-get install libboost-all-dev
	
- 	Serial Library

	The serial library was written by former GAVLab member William Woodall.  It's repository can be found at:

		https://github.com/wjwwood/serial.git

	First clone the serial library.

		git clone https://github.com/wjwwood/serial.git

	Checkout version 1.0.

		cd serial
		git checkout v1.0

	Compile and install the library.

		cmake ./
		make
		sudo make install

### Ublox Interface ###

	To clone the ublox code:

		git clone https://github.com/GAVLab/ublox.git

	Generate MakeFile with cmake and compile the library .

		cd ublox
		mkdir build
		cd build
		cmake ../
		make

	To also build the included executables, invoke cmake with the following option.

		cmake -DUBLOX_BUILD_EXAMPLES=ON ../

	To run one of the generated executables

		cd ~/ublox/bin/
		sudo ./executable [port name] [baudrate]

	For example

		sudo ./record_pseudoranges /dev/ttyUSB0 9600
