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

// gitrev identification from CMake
#ifndef GITREV
#define GITREV "unknown"
#endif

// helper macro to convert value to string
#define STR_HELPER(s) #s    
#define STR(s) STR_HELPER(s)     

// conversion from nanoseconds to seconds
#define NSEC_TO_SEC             (uint64_t)1000000000

// this is just an estaimte used for sizing the samples buffer
// it's not really feasible to reach this sampling rate
#define SAMPLE_RATE_MAX         10000000

// maximum number of pins we support
// no point in having more since only GPIO 0..31 are accessible on the header
#define PINS_MAX                32

// memory mapping
#define GPIO_REG_BASE         0xFE000000  // RPi 4
#define GPIO_BASE             (GPIO_REG_BASE + 0x00200000)

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
  uint64_t capture_len;
  size_t buff_len;
  enum trig_type_e trig;
  int gpio_fd;
  int pins[PINS_MAX];
  unsigned int num_pins;
} conf = {
  .capture_len = NSEC_TO_SEC,
  .buff_len = SAMPLE_RATE_MAX,
  .trig = TRIG_TYPE_RISING,
  .gpio_fd = -1,
  .pins = { 0 },
  .num_pins = 0,
};

// argtable arguments
static struct args_t {
  struct arg_int* pins;
  struct arg_dbl* capture_len;
  struct arg_str* trig_type;
  struct arg_str* labels;
  struct arg_lit* csv;
  struct arg_lit* help;
  struct arg_end* end;
} args;

struct __attribute__((__packed__)) sample_t {
  uint64_t timestamp;
  uint32_t val;
};

static struct sample_t* samples = NULL;

static void sighandler(int signal) {
  (void)signal;
  exit(EXIT_SUCCESS);
}

static void exithandler(void) {
  if(samples) { free(samples); samples = NULL; }
  fflush(stdout);
}

static int digitalReadSingle(int pin) {
  return((*(gpio + 13) & (1 << (pin & 31))) != 0);
}

static uint32_t digitalReadAll(void) {
  return(*(gpio + 13));
}

// helper to convert from timespec to 64-bit count of nanoseconds
static inline uint64_t timespec_to_u64(struct timespec ts) {
  return((uint64_t)(NSEC_TO_SEC * ts.tv_sec + ts.tv_nsec));
}

// helper to convert from 64-bit count of nanoseconds to floating-point seconds
static inline double u64_to_dbl(uint64_t nsec) {
  double timestamp = nsec / NSEC_TO_SEC;
  timestamp += (double)(nsec % NSEC_TO_SEC) / (double)NSEC_TO_SEC;
  return(timestamp);
}

static void wait_for_trigger() {
  int curr;
  int prev = digitalReadSingle(conf.pins[0]);
  bool triggered = false;
  while(!triggered) {
    curr = digitalReadSingle(conf.pins[0]);
    switch(conf.trig) {
      case TRIG_TYPE_IMMEDIATE:
        triggered = true;
        break;
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

static size_t capture() {
  struct timespec ts;
  size_t num_samples = 0;
  timespec_get(&ts, TIME_UTC);
  uint64_t start = timespec_to_u64(ts);
  do {
    timespec_get(&ts, TIME_UTC);
    samples[num_samples].timestamp = timespec_to_u64(ts) - start;
    samples[num_samples].val = digitalReadAll();

    if(num_samples >= conf.buff_len) {
      fprintf(stderr, "Overflow after %.3f seconds!\n", u64_to_dbl(samples[num_samples].timestamp));
      break;
    }

  } while(samples[num_samples++].timestamp < conf.capture_len);
  num_samples--;
  return(num_samples);
}

static int save_csv(size_t num_samples, char* filename) {
  struct timespec ts;
  timespec_get(&ts, TIME_UTC);
  sprintf(filename, "out/pinalyzer_%lu.csv", ts.tv_sec);
  FILE *fptr = fopen(filename, "w");
  if(!fptr) {
    return(EXIT_FAILURE);
  }

  // write the header
  fprintf(fptr, "Time");
  for(unsigned int i = 0; i < conf.num_pins; i++) {
    // use the labels, if provided
    if((unsigned int)args.labels->count >= (i + 1)) {
      fprintf(fptr, ",%s", args.labels->sval[i]);
    } else {
      fprintf(fptr, ",BCM%d", conf.pins[i]);
    }
  }
  fprintf(fptr, "\n;PulseView format spec: t,%db\n", conf.num_pins);

  // write the timestamp and samples
  for(size_t i = 0; i < num_samples; i++) {
    fprintf(fptr, "%.9f", u64_to_dbl(samples[i].timestamp));
    for(unsigned int j = 0; j < conf.num_pins; j++) {
      fprintf(fptr, ",%d", (samples[i].val & (1UL << conf.pins[j])) != 0);
    }
    fprintf(fptr, "\n");
  }

  // close the file and we're done
  fclose(fptr);
  return(EXIT_SUCCESS);
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
    uint32_t val = 0;
    for(unsigned int j = 0; j < conf.num_pins; j++) {
      val |= (((samples[i].val & (1UL << conf.pins[j])) != 0) << j);
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
  fprintf(stdout, "Waiting for trigger\n");
  wait_for_trigger();

  fprintf(stdout, "Running capture\n");
  size_t num_samples = capture();

  fprintf(stdout, "Capture done after %.9f seconds, saving to file\n", u64_to_dbl(samples[num_samples].timestamp));
  double samp_rate = ((double)num_samples/((double)conf.capture_len/(double)NSEC_TO_SEC))/1000000.0;
  char filename[64];
  int ret = EXIT_FAILURE;
  if(args.csv->count) {
    ret = save_csv(num_samples, filename);
    fprintf(stdout, "PulseView format spec: t,%db\n", conf.num_pins);
  } else {
    ret = save_sr(num_samples, filename, samp_rate);
  }

  if(ret == EXIT_SUCCESS) {
    fprintf(stdout, "%lu samples saved to %s\n", num_samples, filename);
    fprintf(stdout, "Average sampling rate is %.3f MSps\n", samp_rate);
  } else {
    fprintf(stderr, "Failed to save %lu samples to %s\n", num_samples, filename);
  }

  return(ret);
}

int main(int argc, char** argv) {
  void *argtable[] = {
    args.pins = arg_intn("p", "pins", NULL, 1, PINS_MAX, "BCMx pins to capture, maximum of " STR(PINS_MAX) ". The first pin will be used as trigger source."),
    args.capture_len = arg_dbl0("l", "capture_len", "sec", "Capture length, defaults to 1.0 second"),
    args.trig_type = arg_str0("t", "trigger", NULL, "Trigger type: r/rising, f/falling, a/any, i/immediate, defaults to rising"),
    args.labels = arg_strn("n", "names", NULL, 1, PINS_MAX, "Signal names for albelling the output, in the order provided pin numbers"),
    args.csv = arg_lit0("c", "csv", "Save output as CSV instead of sigrok PulseView session file"),
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

  // set configuration
  float capture_len_sec = 1.0f;
  if(args.capture_len->count) { capture_len_sec = args.capture_len->dval[0]; }
  conf.capture_len = capture_len_sec * NSEC_TO_SEC;

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

  // allocate memory
  conf.buff_len = SAMPLE_RATE_MAX * capture_len_sec;
  fprintf(stdout, "Allocating %.2f MiB of memory\n", (double)(conf.buff_len * sizeof(struct sample_t))/(double)(1024*1024));
  samples = calloc(conf.buff_len, sizeof(struct sample_t));
  if(!samples) {
    fprintf(stderr, "Failed to allocate samples buffer!\n");
    exitcode = EXIT_FAILURE;
    goto exit;
  }

  // initialize GPIO
  conf.gpio_fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC);
  if(conf.gpio_fd < 0) {
    fprintf(stderr, "Failed to open GPIO device!\n");
    exitcode = EXIT_FAILURE;
    goto exit;
  }

  // access GPIO via memory mapping
  gpio = (uint32_t *)mmap(0, 4*1024, PROT_READ | PROT_WRITE, MAP_SHARED, conf.gpio_fd, GPIO_BASE);
  close(conf.gpio_fd);
  if(gpio == MAP_FAILED) {
    fprintf(stderr, "Failed to map GPIO device!\n");
    exitcode = EXIT_FAILURE;
    goto exit;
  }

  exitcode = run();

exit:
  arg_freetable(argtable, sizeof(argtable)/sizeof(argtable[0]));

  return(exitcode);
}
