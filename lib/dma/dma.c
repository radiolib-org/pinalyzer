/*
  Adapted from https://github.com/fandahao17/Raspberry-Pi-DMA-Tutorial
*/

#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <sys/mman.h>
#include <signal.h>

#include "mailbox.h"
#include "dma.h"
#include "registers.h"

typedef struct DMACtrlReg {
  uint32_t cs;      // DMA Channel Control and Status register
  uint32_t cb_addr; // DMA Channel Control Block Address
} DMACtrlReg;

typedef struct DMAControlBlock {
  uint32_t tx_info;    // Transfer information
  uint32_t src;        // Source (bus) address
  uint32_t dest;       // Destination (bus) address
  uint32_t tx_len;     // Transfer length (in bytes)
  uint32_t stride;     // 2D stride
  uint32_t next_cb;    // Next DMAControlBlock (bus) address
  uint32_t padding[2]; // 2-word padding
} DMAControlBlock;

typedef struct DMAMemHandle {
  void *virtual_addr; // Virutal base address of the page
  uint32_t bus_addr;  // Bus adress of the page, this is not a pointer because it does not point to valid virtual address
  uint32_t mb_handle; // Used by mailbox property interface
  uint32_t size;
} DMAMemHandle;

typedef struct CLKCtrlReg {
  // See https://elinux.org/BCM2835_registers#CM
  uint32_t ctrl;
  uint32_t div;
} CLKCtrlReg;

typedef struct PWMCtrlReg {
  uint32_t ctrl;     // 0x0, Control
  uint32_t status;   // 0x4, Status
  uint32_t dma_cfg;  // 0x8, DMA configuration
  uint32_t padding1; // 0xC, 4-byte padding
  uint32_t range1;   // 0x10, Channel 1 range
  uint32_t data1;    // 0x14, Channel 1 data
  uint32_t fifo_in;  // 0x18, FIFO input
  uint32_t padding2; // 0x1C, 4-byte padding again
  uint32_t range2;   // 0x20, Channel 2 range
  uint32_t data2;    // 0x24, Channel 2 data
} PWMCtrlReg;

static volatile DMACtrlReg *dma_reg;
static volatile PWMCtrlReg *pwm_reg;
static volatile CLKCtrlReg *clk_reg;

static struct dma_conf_t {
  size_t num_samples;
  size_t num_cbs;

  int mailbox_fd;
  DMAMemHandle* dma_cbs;
  DMAMemHandle* dma_samples;
} dma_conf = {
  .num_samples = 0,
  .num_cbs = 0,

  .mailbox_fd = -1,
  .dma_cbs = NULL,
  .dma_samples = NULL,
};

static DMAMemHandle *dma_malloc(unsigned int size) {
  if(dma_conf.mailbox_fd < 0) {
    dma_conf.mailbox_fd = mbox_open();
    assert(dma_conf.mailbox_fd >= 0);
  }

  // Make `size` a multiple of PAGE_SIZE
  size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;

  DMAMemHandle *mem = (DMAMemHandle *)malloc(sizeof(DMAMemHandle));
  // Documentation: https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
  mem->mb_handle = mem_alloc(dma_conf.mailbox_fd, size, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
  mem->bus_addr = mem_lock(dma_conf.mailbox_fd, mem->mb_handle);
  mem->virtual_addr = mapmem(BUS_TO_PHYS(mem->bus_addr), size);
  mem->size = size;

  assert(mem->bus_addr != 0);

  fprintf(stderr, "MBox alloc: %d bytes, bus: %08X, virt: %08X\n", mem->size, mem->bus_addr, (int)mem->virtual_addr);

  return (mem);
}

static void dma_free(DMAMemHandle *mem) {
  if(mem->virtual_addr == NULL) {
    return;
  }

  unmapmem(mem->virtual_addr, PAGE_SIZE);
  mem_unlock(dma_conf.mailbox_fd, mem->mb_handle);
  mem_free(dma_conf.mailbox_fd, mem->mb_handle);
  mem->virtual_addr = NULL;
}

static void *map_peripheral(uint32_t addr, uint32_t size) {
  int mem_fd;

  // Check mem(4) about /dev/mem
  if((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
    perror("Failed to open /dev/mem: ");
    exit(-1);
  }

  uint32_t *result = (uint32_t *)mmap(
      NULL,
      size,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      mem_fd,
      PERI_PHYS_BASE + addr);

  close(mem_fd);

  if (result == MAP_FAILED) {
    perror("mmap error: ");
    exit(-1);
  }

  return(result);
}

static void dma_alloc_buffers() {
  dma_conf.dma_samples = dma_malloc(dma_conf.num_samples * sizeof(uint32_t));
  dma_conf.dma_cbs = dma_malloc(dma_conf.num_cbs * sizeof(DMAControlBlock));
}

static inline void* dma_buff_virt_addr(DMAMemHandle* mem, int i, size_t size) { return mem->virtual_addr + i * size; }
static inline uint32_t dma_buff_bus_addr(DMAMemHandle* mem, int i, size_t size) { return mem->bus_addr + i * size; }

static void dma_init_cbs(bool delay) {
  int cb_idx = 0;
  DMAControlBlock *cb;
  for(size_t i = 0; i < dma_conf.num_samples; i++) {
    // insert sample control block
    cb = (DMAControlBlock*)dma_buff_virt_addr(dma_conf.dma_cbs, cb_idx, sizeof(DMAControlBlock));
    cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP;
    cb->src = PERI_BUS_BASE + GPIO_BASE + GPLEV0;
    cb->dest = dma_buff_bus_addr(dma_conf.dma_samples, i, sizeof(uint32_t));
    cb->tx_len = 4;
    cb_idx++;
    cb->next_cb = dma_buff_bus_addr(dma_conf.dma_cbs, cb_idx, sizeof(DMAControlBlock));

    // insert delay block if needed
    if(delay) {
      cb = (DMAControlBlock*)dma_buff_virt_addr(dma_conf.dma_cbs, cb_idx, sizeof(DMAControlBlock));
      cb->tx_info = DMA_NO_WIDE_BURSTS | DMA_WAIT_RESP | DMA_DEST_DREQ | DMA_PERIPHERAL_MAPPING(5);
      cb->src = dma_buff_bus_addr(dma_conf.dma_cbs, 0, sizeof(DMAControlBlock));
      cb->dest = PERI_BUS_BASE + PWM_BASE + PWM_FIFO;
      cb->tx_len = 4;
      cb_idx++;
      cb->next_cb = dma_buff_bus_addr(dma_conf.dma_cbs, cb_idx, sizeof(DMAControlBlock));
    }
  }

  fprintf(stderr, "DMA init: %lu control blocks, %lu samples\n", dma_conf.num_cbs, dma_conf.num_samples);
}

static void init_hw_clk(int div) {
  // kill the clock if busy
  if(clk_reg->ctrl & CLK_CTL_BUSY) {
    do {
      clk_reg->ctrl = BCM_PASSWD | CLK_CTL_KILL;
    } while(clk_reg->ctrl & CLK_CTL_BUSY);
  }

  // set clock source to PLLD
  clk_reg->ctrl = BCM_PASSWD | CLK_CTL_SRC(CLK_CTL_SRC_PLLD);
  usleep(10);

  // the original clock speed is 750MHZ, divide it
  clk_reg->div = BCM_PASSWD | CLK_DIV_DIVI(div);
  usleep(10);

  // enable the clock
  clk_reg->ctrl |= (BCM_PASSWD | CLK_CTL_ENAB);
}

static void init_pwm(unsigned int range) {
  // reset PWM
  pwm_reg->ctrl = 0;
  usleep(10);
  pwm_reg->status = -1;
  usleep(10);

  // set the range
  pwm_reg->range1 = range;

  // enable PWM DMA, raise panic and dreq thresholds to 15
  pwm_reg->dma_cfg = PWM_DMAC_ENAB | PWM_DMAC_PANIC(15) | PWM_DMAC_DREQ(15);
  usleep(10);

  // clear PWM fifo
  pwm_reg->ctrl = PWM_CTL_CLRF1;
  usleep(10);

  // enable PWM channel 1 and use fifo
  pwm_reg->ctrl = PWM_CTL_USEF1 | PWM_CTL_MODE1 | PWM_CTL_PWEN1;
}

void dma_start() {
  // reset the DMA channel
  dma_reg->cs = DMA_CHANNEL_ABORT;
  dma_reg->cs = 0;
  dma_reg->cs = DMA_CHANNEL_RESET;
  dma_reg->cb_addr = 0;

  dma_reg->cs = DMA_INTERRUPT_STATUS | DMA_END_FLAG;

  // make cb_addr point to the first DMA control block and enable DMA transfer
  dma_reg->cb_addr = dma_buff_bus_addr(dma_conf.dma_cbs, 0, sizeof(DMAControlBlock));
  dma_reg->cs = DMA_PRIORITY(8) | DMA_PANIC_PRIORITY(8) | DMA_DISDEBUG;
  dma_reg->cs |= DMA_WAIT_ON_WRITES | DMA_ACTIVE;
}

void dma_end() {
  // shutdown DMA channel
  dma_reg->cs |= DMA_CHANNEL_ABORT;
  usleep(100);
  dma_reg->cs &= ~DMA_ACTIVE;
  dma_reg->cs |= DMA_CHANNEL_RESET;
  usleep(100);

  // release the memory used by DMA
  dma_free(dma_conf.dma_samples);
  dma_free(dma_conf.dma_cbs);

  free(dma_conf.dma_samples);
  free(dma_conf.dma_cbs);
}

void dma_init(size_t num_samples, unsigned int rate) {
  dma_conf.num_samples = num_samples;
  dma_conf.num_cbs = num_samples;

  // set up access to DMA, PWM and clock registers
  uint8_t *dma_base_ptr = map_peripheral(DMA_BASE, PAGE_SIZE);
  dma_reg = (DMACtrlReg *)(dma_base_ptr + DMA_CHANNEL * 0x100);
  uint8_t *cm_base_ptr = map_peripheral(CM_BASE, CM_LEN);
  clk_reg = (CLKCtrlReg *)(cm_base_ptr + CM_PWM);
  pwm_reg = map_peripheral(PWM_BASE, PWM_LEN);

  // enable rate limiting if the argument is not zero
  if(rate) {
    // calculate the clock divider and PWM timer count
    // TODO calculate both to get some range of frequently used sample rates
    unsigned int div = 10; // 750 MHz / 10 = 75 MHz PWM clock
    unsigned int range = CLK_PLLD_FREQ / (div * rate);  // for 5 MHz rate, range = 15
    dma_conf.num_cbs *= 2;

    init_hw_clk(div);
    usleep(100);

    init_pwm(range);
    usleep(100);
  }

  // allocate buffers based on the number of samples requested by the user
  dma_alloc_buffers();
  usleep(100);

  // initialize control blocks
  dma_init_cbs(rate != 0);
  usleep(100);
}

void* dma_get_samp_ptr(size_t offset) { return(dma_buff_virt_addr(dma_conf.dma_samples, offset, sizeof(uint32_t))); }
