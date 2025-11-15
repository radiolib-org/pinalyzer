# pinalyzer
Raspberry Pi-based logic analyzer. Supports capturing up to 32 GPIO pins and triggering on rising/falling/any edge. Compatible with sigrok file format.

## Dependencies

* cmake >= 3.18
* libzip (can be installed by `sudo apt install libzip-dev`)

## Building

Simply call the `build.sh` script.

## Usage

Start the program by calling `sudo ./build/pinalyzer`. Check the helptext `./build/pinalyzer --help` for all options. At least one pin is required to perform the capture. After starting, the program will wait for the specified trigger on the first pin defined by the `-p` argument, and then capture the state of the specified pins. Multiple pins may be specified, the pin number is the BCM pin number.

Output is a `.sr` file compatible with [sigrok PulseView](https://sigrok.org/wiki/PulseView).

An example call to capture SPI traffic on the [RadioHAT](https://github.com/radiolib-org/RadioHAT) to trigger on falling edge of NSS0 and capture 100 milliseconds of data sampled without rate limiting, with pins labeled with SPI signal names (using sigrok PulseView SPI names):

```
sudo ./build/pinalyzer -tf -l100 -p4 -p17 -p27 -p22 -nCS#0 -nCLK -nMISO -nMOSI
```

## Limitations

Because the program uses memory-mappign via `/dev/mem`, it has to be run as root!

Since this program runs on Linux (a non-realtime OS), the sampling rate is somewhat limited. Sampling of pins is performed by DMA, which in testing on Raspberry Pi 4 can get up to 4 - 5 MHz, which is enough to reliably decode a 1 MHz SPI bus. For sampling rates 1 MHz and more, no throttling is performed. Below this threshold, the sample rate is controlled by a timer.

Since writing samples to file directly would be very slow, the program allocates a working buffer, size of which depends on the capture length and sampling rate. Higher sampling rates with longer captures require larger buffers. As a rule of thumb, the buffer size should not exceed 500k samples (so for example, at 5 Msps, the maximum capture length is about 100 milliseconds).
