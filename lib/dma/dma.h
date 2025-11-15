#ifndef DMA_H
#define DMA_H

#include <stdint.h>

void dma_init(size_t num_samples, unsigned int rate);
void dma_start();
void dma_end();
void* dma_get_samp_ptr(size_t offset);

#endif
