# sdi-12-dr
A library implementing SDI-12 Data Recorder functionality. For more details on the SDI-12 protocol, visit http://sdi-12.org. Quote from the SDI-12 web site:
> SDI-12 stands for serial data interface at 1200 baud. It is a standard to interface battery powered data recorders with micro-processor based sensors designed for environmental data acquisition (EDA)."

## Version
* 0.4 (10 September 2017)

## License
* MIT

## Package
The library is provided as an _xpack_ and can be installed in an Eclipse based project using the attached script (however, the include and source paths must be manually added to the project in Eclipse). For more details on _xpacks_ see https://github.com/xpacks. The installation script requires the helper scripts that can be found at https://github.com/xpacks/scripts.

## Dependencies
The driver depends on the following software package, available as _xpacks_:
* uOS++ (https://github.com/micro-os-plus/micro-os-plus-iii)
* a uOS++ compatible UART driver (e.g. for the STM32F7xxx https://github.com/lixpaulian/stm32f7-uart)

The library has been first developed on an ARM Cortex M7 platform from ST, but it does not depend on a specific microcontroller.

## API Description
The library supports all commands described in the SDI-12 specification, version 1.3 (the new commands introduced in the more recent 1.4 are not yet supported). Following primitives are available:

```c
// open a SDI-12 serial port
bool
sdi12_dr::open (void);

// close an open SDI-12 serial port
void
sdi12_dr::close (void);

// send an acknowledge active command
bool
sdi12_dr::ack_active (char addr);

// send identification command
bool
sdi12_dr::send_id (char addr, char* id, size_t id_len);

// change address command
bool
sdi12_dr::change_address (char addr, char new_addr);

// sample a sensor command using M/C and D or R SDI-12 commands
bool
sdi12_dr::sample_sensor (char addr, sdi12_dr::method_t method, uint8_t index,
                         bool use_crc, float* data, int& max_values);

// asynchronously sample a sensor using the C (concurrent) command
bool
sdi12_dr::sample_sensor_async (char addr, uint8_t index, bool use_crc,
                               float* data, int max_values, bool
                               (*cb) (char, float*, int));

// SDI-12 transparent command; can also be used for sending X commands
bool
sdi12_dr::transparent (char* xfer_buff, int& len);
```

For more details on how to use of these primitives, please see the test files.

## Configuration

Following symbols are used to configure the library:

`SDI_BREAK_LEN` defines the length of the break character (default 20 milliseconds).

`MAX_CONCURENT_REQUESTS` defines the maximum number of concurrent requests when using the `sample_sensor_async` call (default 20). It sets the maximum number of sensors that can be retrieved simultaneously using the "C" (concurrent) command. The `sample_sensor_async` call returns immediatley after querrying a sensor, and the results are delivered through the provided call-back function when the sensor is ready. Between querry and result, the application is free to issue parallel querries to other sensors.

Note that this option may significantly increase the RAM usage: 32 bytes of RAM per concurrent request are used. In addition, a separate "SDI-12 collect" thread will be started with its own stack and RAM requirements. The advantage of the asynchronous primitive comes in handy when there are many sensors to querry, as the data retrieval will be done much faster.

On systems with reduced RAM, you may want to set `MAX_CONCURENT_REQUESTS` to 0; in this case the `sample_sensor_async` primitive will be eliminated and the RAM usage will be reduced. All SDI-12 data retrieval commands, including "C" (concurrent) can be issued using the `sample_sensor` primitive. The sensors will be sampled sequentially, one call at a time.

## Tests
A test suite exercising most of the SDI-12 commands is included. You need an SDI-12 sensor to run the test; you might also need to adapt the test suite to the actual address of the sensor.

After the SDI-12 port is opened, following commands are executed:

* acknowledge active
* sensor identification
* sensor address change (from initial default address 0 to address 1)
* sensor sample (uses SDI-12 "M" and "D" commands)
* asynchronous sensor sample (uses the SDI-12 concurrent "C" command)
* changes the address back to its original default

After the test finishes, the SDI-12 port is closed.

