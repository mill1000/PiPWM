#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <bcm_host.h>

#include "bcm2837_pwm.h"
#include "bcm2837_dma.h"
#include "bcm2837_clock.h"
#include "bcm2837_gpio.h"
#include "bcm2837.h"
#include "log.h"
#include "memory.h"
#include "utils.h"
#include "mailbox.h"
#include "pipwm.h"

#define TAG "PiPWM"

pipwm_channel_t* activeChannels[dma_channel_max];
struct
{
  dma_channel_t dmaChannel;
  memory_physical_t memory;
  double tStep_s;
} timebase;

/**
  @brief  Initialize the PiPWM timebase

  @param  dmaChannel DMA channel to run timebase on
  @param  divi Clock source integer divisor
  @param  divf Clock source fractional divisor
  @param  range Range of PWM peripheral
  @retval none
*/
static void piPwm_initializeTimebase(dma_channel_t dmaChannel, uint16_t divi, uint16_t divf, uint16_t range)
{
  clock_configuration_t clockConfig;
  clockConfig.source = CLOCK_SOURCE_OSCILLATOR;
  clockConfig.mash = CLOCK_MASH_NONE;
  clockConfig.invert = false;
  clockConfig.divi = divi;
  clockConfig.divf = divf;

  LOGD(TAG, "Configuring PWM clock with DIVI: %d, DIVF: %d.", clockConfig.divi, clockConfig.divf);
  clockConfigure(clock_peripheral_pwm, &clockConfig);
  
  clockEnable(clock_peripheral_pwm, true);

  pwm_configuration_t pwmConfig;
  pwmConfig.mode = pwm_mode_mark_space;
  pwmConfig.fifoEnable = true;
  pwmConfig.repeatLast = false;
  pwmConfig.invert = false;
  pwmConfig.silenceBit = 0;

  LOGD(TAG, "Configuring PWM channel with range: %d.", range);
  pwmReset();
  pwmConfigure(pwm_channel_1, &pwmConfig);
  
  // Enable DMA requests when FIFO is below 15 entries
  pwmConfigureDma(true, 15, 15);

  // Configure range and enable
  pwmSetRange(pwm_channel_1, range);
  pwmEnable(pwm_channel_1, true);

  // Allocate memory to contain timing control block
  memory_physical_t memory = memoryAllocatePhysical(sizeof(dma_control_block_t));
  if (memory.address == NULL)
    LOGF(TAG, "Failed to allocate physical memory for PiPWM timebase.");

  // Map the physical memory into our address space
  void* busBase = memory.address;
  void* physicalBase = busBase - bcm_host_get_sdram_address();
  void* virtualBase = memoryMapPhysical((off_t)physicalBase, sizeof(pipwm_control_t));
  if (virtualBase == NULL)
  {
    // Free physical memory
    memoryReleasePhysical(&memory);

    LOGF(TAG, "Failed to map physical memory for PiPWM timebase.");
  }

  // Construct references to PWM peripherals at their bus addresses
  bcm2837_pwm_t* bPwm = (bcm2837_pwm_t*) (BCM2837_BUS_PERIPHERAL_BASE + PWM_BASE_OFFSET);

  // Control blocks reference GPIO, PWM and eachother via bus addresses
  // Application accesses blocks via virtual addresses.
  // Shortcuts to channel data structures in virtual and bus addresses
  dma_control_block_t* bControl = (dma_control_block_t*) busBase;
  dma_control_block_t* vControl = (dma_control_block_t*) virtualBase;

  // Zero-init control block
  memset(vControl, 0, sizeof(dma_control_block_t));

  // Construct control block to repeatedly transfer data to PWM FIFO
  vControl->transferInformation.NO_WIDE_BURSTS = 1;
  vControl->transferInformation.PERMAP = DMA_DREQ_PWM;
  vControl->transferInformation.DEST_DREQ = 1;
  vControl->transferInformation.WAIT_RESP = 1;

  vControl->sourceAddress = bControl; // Don't care
  vControl->destinationAddress = (void*) &bPwm->FIF1;
  vControl->transferLength.XLENGTH = sizeof(uint32_t); // Don't care
  
  vControl->nextControlBlock = bControl;

  // Reset target DMA channel
  dmaReset(dmaChannel);

  // Set control block on target DMA channel and enable
  dmaSetControlBlock(dmaChannel, bControl);
  dmaEnable(dmaChannel, true);

  double divisor = divi + (divf / 4096);
  double tStep_s = (divisor * range) / PIPWM_SOURCE_CLOCK_HZ;

  timebase.dmaChannel = dmaChannel;
  timebase.memory = memory;
  timebase.tStep_s = tStep_s;

  LOGI(TAG, "Timebase configured on DMA channel %d with actual resolution of %g us.", timebase.dmaChannel, timebase.tStep_s * 1e6);
}

/**
  @brief  Initialize the PiPWM system at the target time resolution

  @param  dmaChannel DMA channel to run timebase on
  @param  resolution_s Minimum time resolution in seconds
  @retval none
*/
void piPwm_initialize(dma_channel_t dmaChannel, double resolution_s)
{
  // Initialize BCM peripheral drivers
  bcm2837_init();

  LOGI(TAG, "Initializing PiPWM with target resolution of %g us.", resolution_s * 1e6);

  double divisor = resolution_s * PIPWM_SOURCE_CLOCK_HZ;
  LOGD(TAG, "Clock divisor of %g required for resolution of %g us.", divisor, resolution_s * 1e6);

  uint16_t range = 2;
  while ((divisor / range) > 4096)
    range *= 2;
 
  double divi = 0;
  double divf = 4096 * modf(divisor / range, &divi);

  LOGD(TAG, "Calculated DIVI: %d, DIVF: %d, PWM Range: %d.", (uint16_t) divi, (uint16_t)divf, range);

  // Initialize the timebase with the calculated divisor and range
  piPwm_initializeTimebase(dmaChannel, divi, divf, range);
}

/**
  @brief  Shutdown the PiPWM system, including all channels and the timebase

  @param  none
  @retval none
*/
void piPwm_shutdown()
{
  LOGI(TAG, "Shutting down.");

  // Free any initalized channels
  for (uint8_t i = 0; i < dma_channel_max; i++)
  {
    pipwm_channel_t* channel = activeChannels[i];
    if (channel != NULL)
    {
      LOGD(TAG, "Releasing DMA channel %d.", channel->dmaChannel);
      piPwm_releaseChannel(channel);
    }
  }

  // Shutdown timebase
  dmaEnable(timebase.dmaChannel, false);
  memoryReleasePhysical(&timebase.memory);

  // Disable the PWM timebase
  pwmReset();
  clockEnable(clock_peripheral_pwm, false);
}

/**
  @brief  Generate the DMA controls blocks for the provided channel and buss address

  @param  channel PiPWM channel structure to generate for
  @param  busAddress Bus address of allocated memory
  @retval void* - Bus address of generated control blocks
*/
static void* piPwm_generateControlBlocks(const pipwm_channel_t* channel, void* busAddress)
{
  // Construct references to GPIO peripheral at its bus addresses
  bcm2837_gpio_t* bGpio = (bcm2837_gpio_t*) (BCM2837_BUS_PERIPHERAL_BASE + GPIO_BASE_OFFSET);

  // Control blocks reference GPIO, PWM and eachother via bus addresses
  // Application accesses blocks via virtual addresses.
  // Shortcuts to channel data structures in virtual and bus addresses
  pipwm_control_t* bControl = (pipwm_control_t*) busAddress;
  pipwm_control_t* vControl = channel->vControl;

  // Set data masks
  vControl->setMask = 0; // Starting with no output
  for (uint16_t i = 0; i < channel->steps - 1; i++)
    vControl->clearMask[i] = channel->pinMask;

  // Zero-init all control blocks  
  memset((void*)vControl->controlBlocks, 0, 2 * sizeof(dma_control_block_t));

  // Configure block #1, GPIO set block
  dma_control_block_t* control = &vControl->controlBlocks[0];

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = DMA_DREQ_PWM;
  control->transferInformation.DEST_DREQ = 1;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 1;

  control->sourceAddress = &bControl->setMask;
  control->destinationAddress = (void*) &bGpio->GPSETx[0].SET;
  control->transferLength.XLENGTH = sizeof(uint32_t);

  control->nextControlBlock = &bControl->controlBlocks[1];

  // Configure block #2, GPIO clear block
  control = &vControl->controlBlocks[1];

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = DMA_DREQ_PWM;
  control->transferInformation.DEST_DREQ = 1;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 1;

  control->sourceAddress = &bControl->clearMask[0];
  control->destinationAddress = (void*) &bGpio->GPCLRx[0].CLR;
  control->transferLength.XLENGTH = (channel->steps - 1) * sizeof(uint32_t);

  // Loop block back to start
  control->nextControlBlock = &bControl->controlBlocks[0];

  return bControl->controlBlocks;
}

/**
  @brief  Initalize a PiPWM channel at a given frequency on the pin mask

  @param  dmaChannel DMA channel to use for PWM generation
  @param  pinMask GPIO pin mask of pins to output on
  @param  frequency_Hz Desired PWM frequency in Hz
  @retval pipwm_channel_t* - NULL if error
*/
pipwm_channel_t* piPwm_initalizeChannel(dma_channel_t dmaChannel, gpio_pin_mask_t pinMask, double frequency_Hz)
{
  if (activeChannels[dmaChannel] != NULL)
  {
    LOGE(TAG, "DMA channel %d already in use.", dmaChannel);
    return NULL;
  }

  // Validate that desired frequency won't overflow the DMA blocks
  double tCycle_s = 1 / frequency_Hz;
  uint32_t steps = round(tCycle_s / timebase.tStep_s);
  if (steps >= UINT16_MAX)
  {
    LOGE(TAG, "Requested PWM frequency out of range. Requires %d steps of maximum %d. Increase minimum pulse width.", steps, UINT16_MAX);
    return NULL;
  }

  // Determine size of control data
  size_t length = steps * sizeof(gpio_pin_mask_t) + 2 * sizeof(dma_control_block_t);

  // Allocate physical memory to contain the control data
  memory_physical_t memory = memoryAllocatePhysical(length);
  if (memory.address == NULL)
  {
    LOGE(TAG, "Failed to allocate physical memory for PiPWM channel.");
    return NULL;
  }
	
  // Map the physical memory into our address space
  void* busBase = memory.address;
  void* physicalBase = busBase - bcm_host_get_sdram_address();
  void* virtualBase = memoryMapPhysical((off_t)physicalBase, length);
  if (virtualBase == NULL)
  {
    LOGE(TAG, "Failed to map physical memory for PiPWM channel.");
    
    // Free physical memory
    memoryReleasePhysical(&memory);
    
    return NULL;
  }

  LOGI(TAG, "Configuring DMA channel %d for %g Hz with %d steps.", dmaChannel, frequency_Hz, steps);

  // Configure selected pins as outputs
  gpio_configuration_t gpioConfig;
  gpioConfig.eventDetect = gpio_event_detect_none;
  gpioConfig.function = gpio_function_output;
  gpioConfig.pull = gpio_pull_no_change;
  gpioConfigureMask(pinMask, &gpioConfig);

  // Clear selected pins
  gpioClearMask(pinMask);

  // Allocate memory for the channel structyre
  pipwm_channel_t* channel = malloc(sizeof(pipwm_channel_t));
  if (channel == NULL)
  {
    LOGE(TAG, "Failed to allocate channel buffer.");
    
    // Free physical memory
    memoryReleasePhysical(&memory);
    
    return NULL;
  }

  // Save details to structure
  channel->dmaChannel = dmaChannel;
  channel->memory = memory;
  channel->pinMask = pinMask;
  channel->vControl = virtualBase;
  channel->steps = steps;

  // Generate the DMA control blocks for PWM output
  void* controlBlocks = piPwm_generateControlBlocks(channel, busBase);

  // Reset target DMA channel
  dmaReset(dmaChannel);

  // Set control block on target DMA channel
  dmaSetControlBlock(dmaChannel, controlBlocks);

  // Save reference to channel
  activeChannels[dmaChannel] = channel;

  return channel;
}

/**
  @brief  Release and free the target PiPWM channel

  @param  channel Previously initialize PiPWM channel
  @retval none
*/
void piPwm_releaseChannel(pipwm_channel_t* channel)
{
  // Shut down DMA channel
  dmaEnable(channel->dmaChannel, false);

  // Configure GPIO mask for input
  gpio_configuration_t gpioConfig;
  gpioConfig.eventDetect = gpio_event_detect_none;
  gpioConfig.function = gpio_function_input;
  gpioConfig.pull = gpio_pull_no_change;
  gpioConfigureMask(channel->pinMask, &gpioConfig);

  // Free physical memory
  memoryReleasePhysical(&channel->memory);

  // Mark channel as available
  activeChannels[channel->dmaChannel] = NULL;

  // Free allocated memory for channel
  free(channel);
}

/**
  @brief  Enable or disable the target PiPWM channel

  @param  channel Previously initialize PiPWM channel
  @param  enable Enable or disable channel
  @retval none
*/
void piPwm_enableChannel(const pipwm_channel_t* channel, bool enable)
{
  dmaEnable(channel->dmaChannel, enable);
}

/**
  @brief  Set the output duty cycle of the target channel

  @param  channel Previously initialize PiPWM channel
  @param  dutyCycle PWM duty cycle to set
  @retval none
*/
void piPwm_setDutyCycle(pipwm_channel_t* channel, gpio_pin_mask_t mask, double dutyCycle)
{
  // Force dutyCycle to 0 if desired results in no pulse
  if (floor(dutyCycle * channel->steps) < 1)
    dutyCycle = 0;

  // Force dutyCycle to 1 if desired would result in always full pulse
  if (ceil(dutyCycle * channel->steps) == channel->steps)
    dutyCycle = 1;

  // Mask requested pins by configured pin maks
  gpio_pin_mask_t channelMask = mask & channel->pinMask;

  // Add mask to setMask if pulse should be generated
  if (dutyCycle > 0)
    channel->vControl->setMask |= channelMask;
  else
    channel->vControl->setMask &= ~channelMask;

  // Determine position in sequence where pin should go low
  uint16_t step = dutyCycle * (channel->steps - 1);

  // Update clearMask for desired pulse width
  for (uint16_t i = 0; i < channel->steps; i++)
  {
    if (i < step)
      channel->vControl->clearMask[i] &= ~channelMask;
    else
      channel->vControl->clearMask[i] |= channelMask;
  }
}

/**
  @brief  Set the output pulse width of the target channel

  @param  channel Previously initialize PiPWM channel
  @param  pulseWidth_s Pulse width in seconds
  @retval none
*/
void piPwm_setPulseWidth(pipwm_channel_t* channel, gpio_pin_mask_t mask, double pulseWidth_s)
{
  // Calclate maximum pulse width possible
  double tMax_s = timebase.tStep_s * channel->steps;

  // Calculate duty cycle and pass
  piPwm_setDutyCycle(channel, mask, pulseWidth_s / tMax_s);
}