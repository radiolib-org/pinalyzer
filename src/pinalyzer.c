#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include <sys/mman.h>
#include <fcntl.h>

#include <zip.h>

#include "argtable3/argtable3.h"
#include "dma/dma.h"
#include "dma/registers.h"

// gitrev identification from CMake
#ifndef GITREV
#define GITREV "unknown"
#endif

// helper macro to convert value to string
#define STR_HELPER(s) #s    
#define STR(s) STR_HELPER(s)     

#define SAMPLE_RATE_MAX             5000000
#define SAMPLE_RATE_NO_THROTTLE     1000000

// use the maximum possible sampling rate by default
#define SAMPLE_RATE_DEFAULT         SAMPLE_RATE_MAX

// default capture length in milliseconds
#define CAPTURE_LEN_DEFAULT         50

// maximum number of pins we support
// no point in having more since only GPIO 0..31 are accessible on the header
#define PINS_MAX                    32

// this will later point to memory-mapped GPIO registers
static volatile unsigned int* gpio;

#define SIGROK_FILE_METADATA  \
  "[global]\n" \
  "sigrok version=0.6.0\n\n" \
  "[device 1]\n" \
  "capturefile=logic-1\n" \
  "total probes=%d\n" \
  "samplerate=%.6f MHz\n" \
  "unitsize=1\n" \
  "total analog=0\n"

enum trig_type_e {
  TRIG_TYPE_RISING = 0,
  TRIG_TYPE_FALLING,
  TRIG_TYPE_ANY,
  TRIG_TYPE_IMMEDIATE,
};

// app configuration structure
static struct conf_t {
  int capture_len;
  size_t num_samples;
  enum trig_type_e trig;
  int pins[PINS_MAX];
  unsigned int num_pins;
} conf = {
  .capture_len = CAPTURE_LEN_DEFAULT,
  .num_samples = SAMPLE_RATE_MAX,
  .trig = TRIG_TYPE_RISING,
  .pins = { 0 },
  .num_pins = 0,
};

// argtable arguments
static struct args_t {
  struct arg_int* pins;
  struct arg_int* sample_rate;
  struct arg_int* capture_len;
  struct arg_str* trig_type;
  struct arg_str* labels;
  struct arg_lit* help;
  struct arg_end* end;
} args;

static void sighandler(int signal) {
  (void)signal;
  exit(EXIT_SUCCESS);
}

static void exithandler(void) {
  dma_end();
  fflush(stdout);
}

static int get_gpio(int pin) {
  return((*(gpio + GPLEV0/sizeof(uint32_t)) & (1 << (pin & 31))) != 0);
}

static void wait_for_trigger() {
  bool triggered = false;
  int curr;
  int prev = get_gpio(conf.pins[0]);
  while(!triggered) {
    curr = get_gpio(conf.pins[0]);
    switch((int)conf.trig) {
      case TRIG_TYPE_ANY:
        triggered = (curr != prev);
        break;
      case TRIG_TYPE_RISING:
        triggered = ((prev == 0) && (curr == 1));
        break;
      case TRIG_TYPE_FALLING:
        triggered = ((prev == 1) && (curr == 0));
        break;
    }
    prev = curr;
  }
}

static int zip_add_entry(zip_t *z, char* name, void* data, size_t len) {
  zip_source_t* src = zip_source_buffer(z, NULL, 0, 0);
  if(!src) {
    fprintf(stderr, "Failed to create source buffer: %s\n", zip_strerror(z));
    zip_close(z);
    return(EXIT_FAILURE);
  }

  if(zip_source_begin_write(src) < 0) {
    fprintf(stderr, "zip_source_begin_write failed: %s\n", zip_error_strerror(zip_source_error(src)));
    return(EXIT_FAILURE);
  }
  
  if(zip_source_write(src, data, len) < (zip_int64_t)len) {
    fprintf(stderr, "zip_source_write failed: %s\n", zip_error_strerror(zip_source_error(src)));
    zip_source_rollback_write(src);
    return(EXIT_FAILURE);
  }

  if(zip_source_commit_write(src) < 0) {
    fprintf(stderr, "zip_source_commit_write failed: %s\n", zip_error_strerror(zip_source_error(src)));
    return(EXIT_FAILURE);
  }

  if(zip_file_add(z, name, src, ZIP_FL_OVERWRITE) < 0) {
    fprintf(stderr, "Failed to add %s: %s\n", name, zip_strerror(z));
    return(EXIT_FAILURE);
  }
  
  return(EXIT_SUCCESS);
}

static int save_sr(size_t num_samples, char* filename, double samp_rate) {
  int err = 0;
  zip_error_t zip_err;
  zip_error_init(&zip_err);
  char workbuff[256] = { 0 };

  // create filename based on current time
  struct timespec ts;
  timespec_get(&ts, TIME_UTC);
  sprintf(filename, "out/pinalyzer_%lu.sr", ts.tv_sec);

  // create and open the archive
  zip_t *z = zip_open(filename, ZIP_CREATE | ZIP_TRUNCATE, &err);
  if(!z) {
    zip_error_init_with_code(&zip_err, err);
    fprintf(stderr, "Cannot open zip file: %s\n", zip_error_strerror(&zip_err));
    zip_error_fini(&zip_err);
    return(EXIT_FAILURE);
  }

  // add the metadata file
  int written = sprintf(workbuff, SIGROK_FILE_METADATA, conf.num_pins, samp_rate);
  char* buff_ptr = &workbuff[written];
  for(unsigned int i = 0; i < conf.num_pins; i++) {
    if((unsigned int)args.labels->count >= (i + 1)) {
      written = sprintf(buff_ptr, "probe%d=%s\n", (i + 1), args.labels->sval[i]);
    } else {
      written = sprintf(buff_ptr, "probe%d=BCM%d\n", (i + 1), conf.pins[i]);
    }
    buff_ptr += written;
  }
  zip_add_entry(z, "metadata", workbuff, strlen(workbuff));

  // add the version file (yes, it is just a single number)
  sprintf(workbuff, "2");
  zip_add_entry(z, "version", workbuff, strlen(workbuff));

  // now add all the samples
  zip_source_t* src = zip_source_buffer(z, NULL, 0, 0);
  if(!src) {
    fprintf(stderr, "Failed to create source buffer: %s\n", zip_strerror(z));
    zip_close(z);
    return(EXIT_FAILURE);
  }

  if(zip_source_begin_write(src) < 0) {
    fprintf(stderr, "zip_source_begin_write failed: %s\n", zip_error_strerror(zip_source_error(src)));
    return(EXIT_FAILURE);
  }
  
  // convert all samples to sigrok binary format
  int sample_width = (conf.num_pins + 7) / 8;
  for(size_t i = 0; i < num_samples; i++) {
    uint32_t sample = *(uint32_t*)dma_get_samp_ptr(i);
    uint32_t val = 0;
    for(unsigned int j = 0; j < conf.num_pins; j++) {
      val |= (((sample & (1UL << conf.pins[j])) != 0) << j);
    }
    workbuff[0] = val & 0x000000FFUL;
    workbuff[1] = (val & 0x0000FF00UL) >> 8;
    workbuff[2] = (val & 0x00FF0000UL) >> 16;
    workbuff[3] = (val & 0xFF000000UL) >> 24;

    // this seems very inefficient, but this operation happens entirely in memory, so it is fine
    if(zip_source_write(src, workbuff, sample_width) < (zip_int64_t)sample_width) {
      fprintf(stderr, "zip_source_write failed: %s\n", zip_error_strerror(zip_source_error(src)));
      zip_source_rollback_write(src);
      return(EXIT_FAILURE);
    }
  }

  if(zip_source_commit_write(src) < 0) {
    fprintf(stderr, "zip_source_commit_write failed: %s\n", zip_error_strerror(zip_source_error(src)));
    return(EXIT_FAILURE);
  }

  // dump everything into the same file
  if(zip_file_add(z, "logic-1", src, ZIP_FL_OVERWRITE) < 0) {
    fprintf(stderr, "Failed to add samples: %s\n", zip_strerror(z));
    return(EXIT_FAILURE);
  }

  // all done, close the archive
  if(zip_close(z) < 0) {
    fprintf(stderr, "Failed to close zip archive: %s\n", zip_strerror(z));
    return(EXIT_FAILURE);
  }

  return(EXIT_SUCCESS);
}

static int run() {
  if(conf.trig != TRIG_TYPE_IMMEDIATE) {
    fprintf(stdout, "Waiting for trigger\n");
    wait_for_trigger();
  }

  dma_start();
  fprintf(stdout, "Running capture\n");

  // wait until the DMA is done (1ms more than the capture length)
  usleep((conf.capture_len + 1)*1000UL);

  // convert to sample rate in Msps
  double samp_rate = ((double)conf.num_samples/conf.capture_len)/1000.0;
  char filename[64];
  int ret = save_sr(conf.num_samples, filename, samp_rate);
  if(ret == EXIT_SUCCESS) {
    fprintf(stdout, "%lu samples saved to %s\n", conf.num_samples, filename);
    fprintf(stdout, "Sampling rate %.3f MSps\n", samp_rate);
  } else {
    fprintf(stderr, "Failed to save %lu samples to %s\n", conf.num_samples, filename);
  }

  return(ret);
}

int main(int argc, char** argv) {
  void *argtable[] = {
    args.pins = arg_intn("p", "pins", NULL, 1, PINS_MAX, "BCMx pins to capture, maximum of " STR(PINS_MAX) ". The first pin will be used as trigger source."),
    args.sample_rate = arg_int0("s", "sample_rate", "Sps", "Sample rate, defaults to " STR(SAMPLE_RATE_DEFAULT) " maximum of " STR(SAMPLE_RATE_MAX) ". "\
      "If set to more than " STR(SAMPLE_RATE_NO_THROTTLE) ", then the maximum possible sa sampling rate control above this value is very unreliable."),
    args.capture_len = arg_int0("l", "capture_len", "ms", "Capture length, defaults to 100 milliseconds"),
    args.trig_type = arg_str0("t", "trigger", NULL, "Trigger type: r/rising, f/falling, a/any, i/immediate, defaults to rising"),
    args.labels = arg_strn("n", "names", NULL, 0, PINS_MAX, "Signal names for labeling the output, in the order provided pin numbers"),
    args.help = arg_lit0(NULL, "help", "Display this help and exit"),
    args.end = arg_end(3),
  };

  int exitcode = EXIT_SUCCESS;
  if(arg_nullcheck(argtable) != 0) {
    fprintf(stderr, "%s: insufficient memory\n", argv[0]);
    exitcode = EXIT_FAILURE;
    goto exit;
  }

  int nerrors = arg_parse(argc, argv, argtable);
  if(args.help->count > 0) {
    fprintf(stdout, "RPi GPIO logic analyzer, gitrev " GITREV "\n");
    fprintf(stdout, "Usage: %s", argv[0]);
    arg_print_syntax(stdout, argtable, "\n");
    arg_print_glossary(stdout, argtable,"  %-25s %s\n");
    exitcode = EXIT_SUCCESS;
    goto exit;
  }

  if(nerrors > 0) {
    arg_print_errors(stdout, args.end, argv[0]);
    fprintf(stderr, "Try '%s --help' for more information.\n", argv[0]);
    exitcode = EXIT_FAILURE;
    goto exit;
  }

  atexit(exithandler);
  signal(SIGINT, sighandler);

  // parse pins
  if((args.pins->count < 1) || (args.pins->count > PINS_MAX)) {
    fprintf(stderr, "Invalid number of capture pins: %d\n", args.pins->count);
    exitcode = EXIT_FAILURE;
    goto exit;
  }

  conf.num_pins = args.pins->count;
  for(unsigned int i = 0; i < conf.num_pins; i++) {
    conf.pins[i] = args.pins->ival[i];
  }

  // parse the trigger type
  if(args.trig_type->count) {
    if(strlen(args.trig_type->sval[0]) == 1) {
      switch(args.trig_type->sval[0][0]) {
        case 'r':
          conf.trig = TRIG_TYPE_RISING;
          break;
        case 'f':
          conf.trig = TRIG_TYPE_FALLING;
          break;
        case 'a':
          conf.trig = TRIG_TYPE_ANY;
          break;
        case 'i':
          conf.trig = TRIG_TYPE_IMMEDIATE;
          break;
        default:
          fprintf(stderr, "Uknown trigger type: %c\n", args.trig_type->sval[0][0]);
          exitcode = EXIT_FAILURE;
          goto exit;
      }
    } else {
      if(strcmp(args.trig_type->sval[0], "rising") == 0) {
        conf.trig = TRIG_TYPE_RISING;
      } else if(strcmp(args.trig_type->sval[0], "falling") == 0) {
        conf.trig = TRIG_TYPE_FALLING;
      } else if(strcmp(args.trig_type->sval[0], "any") == 0) {
        conf.trig = TRIG_TYPE_ANY;
      } else if(strcmp(args.trig_type->sval[0], "immediate") == 0) {
        conf.trig = TRIG_TYPE_IMMEDIATE;
      } else {
        fprintf(stderr, "Uknown trigger type: %s\n", args.trig_type->sval[0]);
        exitcode = EXIT_FAILURE;
        goto exit;
      }
    }
  }

  // initialize GPIO
  // TODO this is currently only used for the trigger, rework that to also use DMA
  int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
  if(fd < 0) {
    fprintf(stderr, "Failed to open GPIO device!\n");
    exitcode = EXIT_FAILURE;
    goto exit;
  }

  // access GPIO via memory mapping
  gpio = (uint32_t *)mmap(0, 4*1024, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PERIPH_ADDR(GPIO_BASE));
  close(fd);
  if(gpio == MAP_FAILED) {
    fprintf(stderr, "Failed to map GPIO device!\n");
    exitcode = EXIT_FAILURE;
    goto exit;
  }
  
  // intialize the DMA
  const size_t rate = args.sample_rate->count ? args.sample_rate->ival[0] : SAMPLE_RATE_DEFAULT;
  if(args.capture_len->count) { conf.capture_len = args.capture_len->ival[0]; }
  conf.num_samples = (rate / 1000) * conf.capture_len;
  dma_init(conf.num_samples, (rate >= SAMPLE_RATE_NO_THROTTLE) ? 0 : rate);

  // run the capture
  exitcode = run();

exit:
  arg_freetable(argtable, sizeof(argtable)/sizeof(argtable[0]));

  return(exitcode);
}
