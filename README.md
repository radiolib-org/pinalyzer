# pinalyzer
Raspberry Pi-based logic analyzer. Supports capturing up to 32 GPIO pins and triggering on rising/falling/any edge. Compatible with sigrok file format.

## Dependencies

* cmake >= 3.18
* libzip (can be installed by `sudo apt install libzip-dev`)

## Building

Simply call the `build.sh` script.

## Usage

Start the program by calling `./build/pinalyzer`. Check the helptext `./build/pinalyzer --help` for all options. At least one pin is required to perform the capture. After starting, the program will wait for the specified trigger on the first pin defined by the `-p` argument, and then capture the state of the specified pins. Multiple pins may be specified, the pin number is the BCM pin number.

Output is a .sr file compatible with [sigrok PulseView](https://sigrok.org/wiki/PulseView). It is also possible to save output as CSV file by using the optional argument `-c`, first column is time since capture in seconds, the following columns are logic states. However, the resulting files will be very large, so this is not recommended.

An example call to capture SPI traffic on the [RadioHAT](https://github.com/radiolib-org/RadioHAT) to trigger on falling edge of NSS0 and capture 1 second of data, with pins labeled with SPI signal names (using sigrok PulseView SPI names) and output to sigrok PulseView session file:

```
./build/pinalyzer -tf -p4 -p17 -p27 -p22 -nCS#0 -nCLK -nMISO -nMOSI
```

## Limitations

Since this program runs on Linux (a non-realtime OS), the sampling rate cannot be guaranteed, and jumps in the timebase are common. No throttling is performed either, and the sampling will be done as fast as the program allows. In testing on Raspberry Pi 4, this is nearly 8 MHz, which is more than enough to capture up to 2 MHz SPI.

Since writing samples to file directly would be very slow, the program allocates a working buffer, size of which depends on the capture length. It is not advised to create long (10s+) capture files!

