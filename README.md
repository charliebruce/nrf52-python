# nrf52-python

Library to interact with an nRF52 from Python using just SWD lines.

## How it Works

This library directly reads to / writes from the nRF52's registers, and thereby controls its peripherals, using a debugger connected to the SWD lines.

## Use Cases

* Automated testing of PCBs during manufacture
* Gaining an understanding of / testing peripherals / bringing up new PCBs quickly
* Testing reference drivers
* Datalogging from custom PCBs

## Limitations

* This approach is very slow (in the order of milliseconds to toggle a pin).
* There's an operating system in the loop, so will not be suitable for real-time constrained applications.
* There may often be better ways to achieve the same goal. For example, for programming an SPI Flash IC, you could use this method to program the Flash via the nRF52. In practice though, you may achieve faster cycle times by adding test points to the SPI bus, and using something like `flashrom` to write the IC from a Raspberry Pi or dedicated programmer.
* There are numerous errata in the nRF52 chip family. Nordic's SDK/drivers do a good job at working around / highlighting these issues, but this approach does not benefit from Nordic's work.
* This code is only lightly tested (and all testing is manual), and should be considered alpha-quality.

## Dependencies

Uses [pylink-square](https://pypi.org/project/pylink-square/) for SWD connection to the nRF52.
