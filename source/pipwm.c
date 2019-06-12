#include <math.h>
#include <stdlib.h>
#include <string.h>
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

#define TAG "MAIN"


static pipwm_channel_t* activeChannels[dma_channel_max];
static double tStep_s = 0;

/**
  @brief  Initialize the PiPWM system, include peripheral drivers and timebase

  @param  divi Clock source integer divisor
  @param  divf Clock source fractional divisor
  @param  range Range of PWM peripheral
  @retval none
*/
void piPwm_initialize(uint16_t divi, uint16_t divf, uint16_t range)
{
  // Initialize BCM peripheral drivers
  bcm2837_init();

  uint32_t dmaMask = mailboxGetDmaChannelMask();

  LOGI(TAG, "Initializing PiPWM.");
  LOGI(TAG, "Available DMA Channels:");
  for (uint32_t i = 0; i < 32; i++)
  {
    if (dmaMask & (1 << i))
      LOGI(TAG, "Channel %d", i);
  }

  clock_configuration_t clockConfig;
  clockConfig.source = CLOCK_SOURCE_OSCILLATOR;
  clockConfig.mash = CLOCK_MASH_NONE;
  clockConfig.invert = false;
  clockConfig.divi = divi;
  clockConfig.divf = divf;

  LOGI(TAG, "Configuring PWM clock with DIVI: %d, DIVF: %d.", clockConfig.divi, clockConfig.divf);
  clockConfigure(clock_peripheral_pwm, &clockConfig);
  
  clockEnable(clock_peripheral_pwm, true);

  pwm_configuration_t pwmConfig;
  pwmConfig.mode = pwm_mode_mark_space;
  pwmConfig.fifoEnable = true;
  pwmConfig.repeatLast = false;
  pwmConfig.invert = false;
  pwmConfig.silenceBit = 0;

  LOGI(TAG, "Configuring PWM channel with range: %d.", range);
  pwmReset();
  pwmConfigure(pwm_channel_1, &pwmConfig);
  
  // Enable DMA requests when FIFO is below 15 entries
  pwmConfigureDma(true, 15, 15);

  // Configure range and enable
  pwmSetRange(pwm_channel_1, range);
  pwmEnable(pwm_channel_1, true);

  double divisor = divi + (divf / 4096);
  tStep_s = (divisor * range) / 19.2e6;
  LOGI(TAG, "Timebase configured for a delta T of %g us.", tStep_s * 1e6);
}

/**
  @brief  Shutdown the PiPWM system, including all channels and the timebase

  @param  none
  @retval none
*/
void piPwm_shutdown()
{
  // Free any initalized channels
  for (uint8_t i = 0; i < dma_channel_max; i++)
  {
    if (activeChannels[i] != NULL)
      piPwm_freeChannel(activeChannels[i]);
  }

  // Disable the PWM timebase
  pwmReset();
  clockEnable(clock_peripheral_pwm, false);
}

/**
  @brief  Generate the DMA controls blocks for the provided channel and buss address

  @param  channel PiPWM channel structure to generate for
  @param  busAddress Bus address of allocated memory
  @retval none
*/
static void piPwm_generateControlBlocks(const pipwm_channel_t* channel, void* busAddress)
{
  // Construct references to GPIO and PWM peripherals at their bus addresses
  bcm2837_pwm_t* bPwm = (bcm2837_pwm_t*) (BCM2837_BUS_PERIPHERAL_BASE + PWM_BASE_OFFSET);
  bcm2837_gpio_t* bGpio = (bcm2837_gpio_t*) (BCM2837_BUS_PERIPHERAL_BASE + GPIO_BASE_OFFSET);

  // Shortcuts to channel data structures in virtual and bus addresses
  pipwm_control_t* bControl = (pipwm_control_t*) busAddress;
  pipwm_control_t* vControl = channel->vControl;

  // Set data masks
  vControl->setMask = 0; // Starting with no output
  vControl->clearMask = channel->pinMask;

  // Zero-init all control blocks  
  memset((void*)vControl->controlBlocks, 0, 4 * sizeof(dma_control_block_t));

  // Control blocks reference GPIO, PWM and eachother via bus addresses
  // Application accesses blocks via virtual addresses.

  // Configure block #1, GPIO set block
  dma_control_block_t* control = &vControl->controlBlocks[0];

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 1;

  control->sourceAddress = &bControl->setMask;
  control->destinationAddress = (void*) &bGpio->GPSETx[0].SET;
  control->transferLength.XLENGTH = sizeof(uint32_t);

  control->nextControlBlock = &bControl->controlBlocks[1];

  // Configure block #2, positive pulse width delay
  control = &vControl->controlBlocks[1];

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = DMA_DREQ_PWM;
  control->transferInformation.SRC_DREQ = 1;
  control->transferInformation.DEST_DREQ = 1;
  control->transferInformation.WAIT_RESP = 1;

  control->sourceAddress = &bControl->setMask;
  control->destinationAddress = (void*) &bPwm->FIF1;
  control->transferLength.XLENGTH = 0;

  control->nextControlBlock = &bControl->controlBlocks[2];

  // Configure block #3, GPIO clear block
  control = &vControl->controlBlocks[2];

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.WAIT_RESP = 1;
  control->transferInformation.SRC_INC = 1;

  control->sourceAddress = &bControl->clearMask;
  control->destinationAddress = (void*) &bGpio->GPCLRx[0].CLR;
  control->transferLength.XLENGTH = sizeof(uint32_t);

  control->nextControlBlock = &bControl->controlBlocks[3];

  // Configure block #4, negative pulse width delay
  control = &vControl->controlBlocks[3];

  control->transferInformation.NO_WIDE_BURSTS = 1;
  control->transferInformation.PERMAP = DMA_DREQ_PWM;
  control->transferInformation.SRC_DREQ = 1;
  control->transferInformation.DEST_DREQ = 1;
  control->transferInformation.WAIT_RESP = 1;

  control->sourceAddress = &bControl->clearMask;
  control->destinationAddress = (void*) &bPwm->FIF1;
  control->transferLength.XLENGTH = channel->steps * sizeof(uint32_t);

  // Loop block back to start
  control->nextControlBlock = &bControl->controlBlocks[0];
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

  // Allocate physical memory to contain the control data
  memory_physical_t memory = memoryAllocatePhysical(sizeof(pipwm_control_t));
  if (memory.address == NULL)
  {
    LOGE(TAG, "Failed to allocate physical memory for PiPWM channel.");
    return NULL;
  }
	
  // Map the physical memory into our address space
  void* busBase = memory.address;
  void* physicalBase = busBase - bcm_host_get_sdram_address();
  void* virtualBase = memoryMapPhysical((off_t)physicalBase, sizeof(pipwm_control_t));
  if (virtualBase == NULL)
  {
    LOGE(TAG, "Failed to map physical memory for PiPWM channel.");
    return NULL;
  }

  // Validate that desired frequency won't overflow the DMA blocks
  double tCycle_s = 1 / frequency_Hz;
  uint32_t steps = round(tCycle_s / tStep_s);
  if (steps >= UINT16_MAX)
  {
    LOGE(TAG, "Requested PWM frequency out of range. Requires %d steps of maximum %d. Increase minimum pulse width.", steps, UINT16_MAX);
    return NULL;
  }

  LOGI(TAG, "Configuring DMA channel %d for %f Hz with %d steps.", dmaChannel, frequency_Hz, steps);

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
    return NULL;
  }

  // Save details to structure
  channel->dmaChannel = dmaChannel;
  channel->memory = memory;
  channel->pinMask = pinMask;
  channel->vControl = virtualBase;
  channel->steps = steps;

  // Generate the DMA control blocks for PWM output
  piPwm_generateControlBlocks(channel, busBase);

  // Reset target DMA channel
  dmaReset(dmaChannel);

  // Set control block on target DMA channel
  dmaSetControlBlock(dmaChannel, busBase);

  // Save reference to channel
  activeChannels[dmaChannel] = channel;

  return channel;
}

/**
  @brief  Release and free the target PiPWM channel

  @param  channel Previously initialize PiPWM channel
  @retval none
*/
void piPwm_freeChannel(pipwm_channel_t* channel)
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
  @brief  Set the output duty cycle of the desired channel

  @param  channel Previously initialize PiPWM channel
  @param  ratio PWM duty cycel to set
  @retval none
*/
void piPwm_setRatio(pipwm_channel_t* channel, double ratio)
{
  // Force ratio to 0 if desired results in no pulse
  if (round(ratio * channel->steps) < 1)
    ratio = 0;

  // Force ratio to 1 if desired would result in always full pulse
  if (round(ratio * channel->steps) == channel->steps)
    ratio = 1;

  // Adjust pinmaks as necessary to eliminate glitches
  channel->vControl->setMask = (ratio > 0) ? channel->pinMask : 0;
  channel->vControl->clearMask = (ratio < 1) ? channel->pinMask : 0;

  // Update delay structures for new ratio
  channel->vControl->controlBlocks[1].transferLength.XLENGTH = ratio * channel->steps * sizeof(uint32_t);
  channel->vControl->controlBlocks[3].transferLength.XLENGTH = (1 - ratio) * channel->steps * sizeof(uint32_t);
}

int main()
{
  double frequency_Hz = 100;
  double tCycle_s = 1 / frequency_Hz;
  double tStep_s = 10e-6;

  uint32_t steps = round(tCycle_s / tStep_s);
  assert(steps < UINT16_MAX);
// /
  LOGI(TAG, "Delta T: %g us, Clock: %g Hz, Steps: %d", tStep_s * 1e6, frequency_Hz, steps);
// /
   double srcClock_Hz = 19.2e6;
// /
  double divider = srcClock_Hz * tStep_s;
  LOGI(TAG, "Target divisor: %g", divider);
// /
  // /uint32_t minDivi = 0;
  // /uint32_t minRange = 0;
  // /double minError = DBL_MAX;
  // /double tStepCalc_s = 0.0;
  // /uint32_t range = 2;
  // /uint32_t divi = divider / 2;
  // /while (divi > 1)
  // /{
  // /  divi = divider / range;
  // /  tStepCalc_s = (divi * range) / srcClock_Hz;
// /
  // /  double error = fabs(tStep_s - tStepCalc_s);
  // /  
  // /  LOGI(TAG, "DIVI: %d, Range: %d, Error: %g", divi, range, error);
// /
  // /  if (error < minError)
  // /  {
  // /    minError = error;
  // /    minRange = range;
  // /    minDivi = divi;
  // /  }
// /
  // /  if (error < 1e-9)
  // /    break;
// /
  // /  range++;
  // /}
  // /LOGI(TAG, "Best DIVI: %d, Best Range: %d, Min error: %g", minDivi, minRange, minError);
  // /tStepCalc_s = srcClock_Hz / (minDivi * minRange);
  // /LOGI(TAG, "Clk Divider: %d, Range: %d, Actual delta T: %g us", divi, range, 1e6/tStepCalc_s);
// /
// /
  // /
 

  double integral = 0;
  double frac = modf(divider / 2, &integral);
  
  piPwm_initialize(integral, frac * 4096, 2);
 
  double ratio = .1;
  gpio_pin_mask_t mask = 1 << 12;


  pipwm_channel_t* channel = piPwm_initalizeChannel(dma_channel_13, mask, 100);
  if (channel == NULL)
    LOGF(TAG, "Failed to initalize PiPWM channel");

  
  piPwm_enableChannel(channel, true);

  //while(1)
  {
    sleep(1);
    piPwm_setRatio(channel, .25);
    sleep(1);
    piPwm_setRatio(channel, .75);
    
  }


  piPwm_freeChannel(channel);

  piPwm_shutdown();

  return 0;
}