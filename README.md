# pinalyzer
Raspberry Pi-based logic analyzer. Supports capturing up to 32 GPIO pins and triggering on rising/falling/any edge.

## Dependencies

* cmake >= 3.18

## Building

Simply call the `build.sh` script.

## Usage

Start the program by calling `./build/pinalyzer`. Check the helptext `./build/pinalyzer --help` for all options. At least one pin is required to perform the capture. After starting, the program will wait for the specified trigger on the first pin defined by the `-p` argument, and then capture the state of the specified pins. Multiple pins may be specified, e.g. `pinalyzer -p4 -p17`. The pin number is the BCM pin number.

Output is a CSV file, first column is time since capture in seconds, the following columns are logic states. The output file can then be loaded into [sigrok PulseView](https://sigrok.org/wiki/PulseView). To open the file, go to Open -> Import Comma-separated values, and enter the format specifier returned at the end of the pinalyzer capture run. 

## Limitations

Since this program runs on Linux (a non-realtime OS), the sampling rate cannot be guaranteed, and jumps in the timebase are common. No throttling is performed either, and the sampling will be done as fast as the program allows. In testing on Raspberry Pi 4, this is nearly 8 MHz, which is more than enough to capture up to 2 MHz SPI.

Since writing samples to file directly would be very slow, the program allocates a working buffer, size of which depends on the capture length. It is not advised to create long (10s+) capture files!

