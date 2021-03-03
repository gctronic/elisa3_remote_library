# Elisa-3 remote library
This library simplify the implementation of applications that need to control one or more robots remotely thorugh the radio base-station connected to the computer. The library provides functions to set all actuators and receive sensors data from the robots.<br/>
The project is built with [Code::Blocks (mingw)](http://www.codeblocks.org/) and requires [libusb-1.0](http://www.libusb.info) (<code>libusb-1.0.21</code> is included in the repo) since it is used to communicate with the radio base-station.

Compilation on Windows:
*  within the <code>libusb-1.0</code> package you'll find the header <code>libusb.h</code> (under <code>/include/libusb-1.0/</code>) and the library <code>libusb-1.0.a</code> (in this case under <code>/MinGW32/static/</code> since we use MinGW) that should be copied or referenced in the compiler headers/libraries
* add the linker flag <code>-lusb-1.0</code> in the project in order to build successfully
* the Code::Blocks project provided has already these configurations set <br/>

Compilation on Linux / Mac OS X:
* required libraries: <code>libusb-1.0</code>, <code>libusb-1.0-dev</code>, <code>libncurses5</code>, <code>libncurses5-dev</code>
* build: under the <code>/linux</code> folder within the project directory there is a makefile, simply type <code>make clean && make</code> on a terminal to build the library
