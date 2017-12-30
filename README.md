# sdi-12-dr
A class implementing SDI-12 Data Recorder functionality. For more details on the SDI-12 protocol, visit http://sdi-12.org. Quote from the SDI-12 web site:
> SDI-12 stands for serial data interface at 1200 baud. It is a standard to interface battery powered data recorders with micro-processor based sensors designed for environmental data acquisition (EDA)."

## Version
* 0.8 (30 December 2017)

## License
* MIT

## Package
The class is provided as an _xpack_ and can be installed in an Eclipse based project using the attached script (however, the include and source paths must be manually added to the project in Eclipse). For more details on _xpacks_ see https://github.com/xpacks. The installation script requires the helper scripts that can be found at https://github.com/xpacks/scripts.

## Dependencies
This software depends on the following packages, available as _xpacks_:
* uOS++ (https://github.com/micro-os-plus/micro-os-plus-iii)
* a uOS++ compatible UART driver (e.g. for the STM32F7xxx https://github.com/lixpaulian/stm32f7-uart)

The class has been first developed on an ARM Cortex M7 platform from ST, but it does not depend on a specific microcontroller.

## API Description
Please note that a massive API change was done between versions 0.5 and 0.6 and in this process the compatibility was lost. This was necessary for several reasons, but most importantly because the SDI-12 class is now derived from the generic `dacq` class. In the grand scheme of C++ things, the `dacq` class is (and will be) used for other sensor interface/protocol implementations.

The sdi-12-dr class implements several primitives that may be used to resolve all the commands described in the SDI-12 specification, version 1.3 (the new commands introduced in the more recent 1.4 SDI-12 specification are not supported).

Following primitives are available:

```c
// open a SDI-12 serial port
bool
open (speed_t baudrate, uint32_t c_size, uint32_t parity, uint32_t rec_timeout);

// close an open SDI-12 serial port
void
sdi12_dr::close (void);

// send identification command
bool
get_info (int id, char* ver, size_t len);

// change sensor address
bool
change_id (int id, int new_id);

// sample a sensor command using M/C and D, or R SDI-12 commands
bool
retrieve (dacq_handle_t* dacqh);

// executes a transparent SDI-12 transaction (sends a request and returns the answer)
bool
transparent (char* xfer_buff, int& len);

// get driver version number
void
get_version (uint8_t& version_major, uint8_t& version_minor);

```

For more details on how to use of these primitives, please see dacq.h header file and the test files.

## Configuration

Following symbols are used to configure the software:

`SDI_BREAK_LEN` defines the length of the break character (default 20 milliseconds).

`MAX_CONCURRENT_REQUESTS` defines the maximum number of concurrent requests (default 10) when using the `retrieve` call in conjunction with the SDI-12 "C" (or "CC") command. It sets the maximum number of sensors that can be retrieved simultaneously. The `retrieve` call returns in this case immediatley after querrying a sensor, and the results are delivered through the provided call-back function after the sensor is ready. Between querry and result, the application is free to issue parallel ("concurrent") querries to other sensors.

Note that this option may significantly increase the RAM usage: 36 bytes of RAM per concurrent request are used. In addition, a separate "SDI-12 collect" thread will be started with its own stack and RAM requirements. The advantage of the asynchronous primitive comes in handy when there are many sensors to querry, as by paralleling the requests, the data retrieval will be done much faster.

On systems with reduced RAM, you may want to set `MAX_CONCURRENT_REQUESTS` to 0. All SDI-12 data retrieval commands, including "C" and "CC" (concurrent) can still be issued using the `retrieve` primitive; however, in this case the concurrent command "C" will be sequentially executed.

## Tests
A test suite exercising most of the SDI-12 commands is included. You need an SDI-12 sensor to run the test; you might also need to adapt the test suite to the actual address of the sensor.

After the SDI-12 port is opened, following commands are executed:

* acknowledge active
* sensor identification
* sensor address change (from initial default address 0 to address 1)
* sensor sample (uses SDI-12 "M" and "D" commands)
* asynchronous sensor sample for two different sensors using the SDI-12 concurrent "C" command
* sensor address change (back to its original default)

After the test finishes, the SDI-12 port is closed.

