![GitHub package.json version](https://img.shields.io/github/package-json/v/lixpaulian/dacq)
![GitHub Tag](https://img.shields.io/github/v/tag/lixpaulian/dacq)
![GitHub License](https://img.shields.io/github/license/lixpaulian/dacq)

# sdi-12-dr
A class implementing SDI-12 Data Recorder functionality. For more details on the SDI-12 protocol, visit http://sdi-12.org. Quote from the SDI-12 web site:
> SDI-12 stands for serial data interface at 1200 baud. It is a standard to interface battery powered data recorders with micro-processor based sensors designed for environmental data acquisition (EDA)."

## Package
The class is provided as an **xPack** (for more details on xPacks see https://xpack.github.io). It can be installed in a project using either `xpm` or the attached script. Of course, it can be installed without using the xPacks tools, either by linking the class as a Git submodule or by copying it in your project, but then updating it later might be more difficult.

Note that the xPacks project evolved with the time. Initially it was based on shell scripts, but the current version is based on a set of utilities, `xpm` and a JSON description file. You will still find the `xpacks-helper.sh` script in the `scripts` subdirectory, but it is not recommened as it is deprecated and will not be supported in the future. Instead use the procedure described below.

To install the package using `xpm` you must make sure that you have already `nodejs` and `xpm` installed on your computer (see also [xPack install](https://xpack.github.io/install/)). Then, in your project directory issue the commands:

```sh
cd my-project
xpm init # Add a package.json if not already present
xpm install github:lixpaulian/dacq#v1.5.4 --copy
```

Note: Without `--copy`, the default is to create a link to a read-only instance of the package in the `xpm` central store.

## Dependencies
This software depends on the following packages, available as xPacks:
* µOS++ (https://github.com/micro-os-plus/micro-os-plus-iii)
* a µOS++ compatible UART driver (e.g. for the STM32F7xxx https://github.com/lixpaulian/stm32f7-uart)

The class has been first developed on an ARM Cortex M7 platform from ST, but it does not depend on a specific microcontroller.

## API Description
The sdi-12-dr class implements several primitives that may be used to resolve all the commands described in the SDI-12 specification, version 1.3 (the new commands introduced in the more recent 1.4 SDI-12 specification are not supported).

The generic `dacq` class defines following primitives (a specific class, and the sdi-12-dr is no exception, might implement only a subset of these):

```c
// open a SDI-12 serial port
bool
open (speed_t baudrate, uint32_t c_size, uint32_t parity, uint32_t rec_timeout);

// provides a direct connection to the DACQ port
void
direct (int fildes, int timeout);

// close an open SDI-12 serial port
void
close (void);

// check if an operation is under way, i.e. if the system is busy
bool
is_busy (void);

// get driver version number
void
get_version (uint8_t& version_major, uint8_t& version_minor, uint8_t& version_patch);

// send identification command
bool
get_info (int id, char* ver, size_t len);

// sample a sensor command using M/C and D, or R SDI-12 commands
bool
retrieve (dacq_handle_t* dacqh);

// executes a transparent SDI-12 transaction (sends a request and returns the answer)
bool
transparent (char* xfer_buff, int& len);

// change sensor address
bool
change_id (int id, int new_id);

// set the acquisition/sampling interval
bool
set_acq_interval (int interval);

// get the current acquisition/sampling interval
bool
get_acq_interval (int& interval);

// set the date/time of the sensor/logger
bool
set_date (time_t date);

// get the current date of the sensor/logger
time_t
get_date (void);

// abort a running operation (e.g. retrieve)
bool
abort (void);

// set a function to dump transactions with the sensor(s), e.g. for protocol debugging.
void
set_dump_fn (void (*dump_fn) (char*));

// disable dumping sensor transactions.
void
unset_dump_fn (void);


```

Most functions return a boolean, if it is `true`, then the function was successful, otherwise it failed. For more information on why it failed, the caller can use the `error` structure containing the error type and the error text (see below).

```c
typedef struct err_
  {
    int error_number;
    const char* error_text;
  } err_t;
```

For more details on how to use of these primitives, please see dacq.h header file and the test files.

## Configuration

Following symbols are used to configure the software:

`SDI_BREAK_LEN` defines the length of the break character (default 20 milliseconds).

`MAX_CONCURRENT_REQUESTS` defines the maximum number of concurrent requests (default 10) when using the `retrieve` call in conjunction with the SDI-12 "C" (or "CC") command. It sets the maximum number of sensors that can be retrieved simultaneously. The `retrieve` call returns in this case immediatley after querrying a sensor, and the results are delivered through the provided call-back function after the sensor is ready. Between querry and result, the application is free to issue parallel ("concurrent") querries to other sensors.

Note that this option may significantly increase the RAM usage: 36 bytes of RAM per concurrent request are used; for 10 concurrent sensor requests that would mean 360 bytes of RAM. In addition, a separate "SDI-12 collect" thread will be started with its own stack and RAM requirements. The advantage of the asynchronous primitive comes in handy when there are many sensors to querry, as by paralleling the requests, the data retrieval will be done much faster.

On systems with reduced RAM, you may want to set `MAX_CONCURRENT_REQUESTS` to 0. All SDI-12 data retrieval commands, including "C"/"CC" (concurrent) can still be issued using the `retrieve` primitive; however, in this case the concurrent commands "C"/"CC" will be sequentially executed too.

Note however, if you set `MAX_CONCURRENT_REQUESTS` to 2 or more, all the sensors on the bus must be interrogated with (and support) the "C"/"CC" command. You cannot mix concurrent with non-concurrent requests on the same SDI-12 bus! 

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

