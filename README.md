# nrf52-python

Library to interact with an nRF52 from Python using just SWD lines.

## How it Works

This library directly reads to / writes from the nRF52's registers, and thereby controls its peripherals, using a debugger connected to the SWD lines.

## Use Cases

* Automated testing of PCBs during manufacture
* Gaining an understanding of / testing peripherals / bringing up new PCBs quickly
* Testing reference drivers
* Datalogging from custom PCBs

## Dependencies

Uses [pylink-square](https://pypi.org/project/pylink-square/) for SWD connection to the nRF52.
