#include <stddef.h>
#include <assert.h>

#include "bcm2837_dma.h"

static void* dma = NULL;

/**
  @brief  Initalize the DMA object at the given base address

  @param  base Base address of DMA peripheral
  @retval none
*/
void dmaInit(void* base)
{
  assert(base != NULL);
  assert(dma == NULL);

  dma = base;
}

/**
  @brief  Fetch the channel registers for the supplied channel index.

  @param  channel DMA channel number
  @retval bcm2837_dma_channel_t*
*/
static bcm2837_dma_channel_t* dmaGetChannel(dma_channel_t channel)
{
  assert(dma != NULL);
  assert(channel < dma_channel_max);

  return (bcm2837_dma_channel_t*) dma + (channel * DMA_CHANNEL_OFFSET);
}
