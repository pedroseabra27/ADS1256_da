This program is designed to acquire data from ADS1256 ADC
board, connected to Raspberry Pi, show this data on screen and
store to file.

Different CPS, modes and channels combinations is supported.

Notes:
 - Default Chip Select (CS) pin now set to CE0 (physical pin 24 / GPIO8) which matches most ADS1256 + DAC8552 "High-Precision AD/DA" HATs.
 - DRDY expected on pin 11 (GPIO17), RST on pin 12 (GPIO18).
 - SPI mode: MODE1, MSB first.
 - Use -P for a short probe; add -X (after local update including diagnostics) to print GPIO states.


