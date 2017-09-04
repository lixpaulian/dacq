# sdi-12-dr
A library implementing SDI-12 Data Recorder functionality. For more details on the SDI-12 protocol, visit http://sdi-12.org. Quote from the SDI-12 web site:
> SDI-12 stands for serial data interface at 1200 baud. It is a standard to interface battery powered data recorders with micro-processor based sensors designed for environmental data acquisition (EDA)."

## Version
* 0.3 (4 September 2017)

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
TBD.

## Tests
A test suite exercising most of the SDI-12 commands is included. You need an SDI-12 sensor to run the test; you might also need to adapt the test suite to the actual address of the sensor.

This is work in progress.
