#include <stddef.h>
#include <assert.h>

#include <bcm2837_clock.h>

static void* clock = NULL;

/**
  @brief  Initalize the Clock object at the given base address

  @param  base Base address of Clock peripheral
  @retval none
*/
void clockInit(void* base)
{
  assert(base != NULL);
  assert(clock == NULL);

  clock = base;
}

/**
  @brief  Fetch the clock configuration structure for the given peripheral

  @param  peripheral Target peripheral to fetch clock config for
  @retval bcm2837_clock_t*
*/
static bcm2837_clock_t* clockGetPeripheralClock(clock_peripheral_t peripheral)
{
  assert(clock != NULL);
  assert(peripheral < clock_peripheral_max);

  uint32_t offset = 0;
  switch (peripheral)
  {
    case clock_peripheral_gp0:
      offset = CLOCK_GP0_OFFSET;
      break;

    case clock_peripheral_gp1:
      offset = CLOCK_GP1_OFFSET;
      break;

      case clock_peripheral_gp2:
      offset = CLOCK_GP2_OFFSET;
      break;

    case clock_peripheral_pcm:
      offset = CLOCK_PCM_OFFSET;
      break;

    case clock_peripheral_pwm:
      offset = CLOCK_PWM_OFFSET;
      break;

    default:
      return NULL;
  }

  return (bcm2837_clock_t*) (clock + offset);
}

/**
  @brief  Enable or disable the target peripheral's clock

  @param  peripheral Target peripheral clock to enable
  @param  enable Enable the peripheral clock
  @retval void
*/
void clockEnable(clock_peripheral_t peripheral, bool enable)
{
  bcm2837_clock_t* clock = clockGetPeripheralClock(peripheral);

  assert(clock != NULL);

  // Read existing control register
  clock_control_t control = clock->CTL;
  RMB();

  // Set password and enable bits
  control.PASSWD = CLOCK_MANAGER_PASSWORD;
  control.ENAB = enable;

  // Write updated register to device
  WMB();
  clock->CTL = control;
}

/**
  @brief  Wait for the peripheral clock to be idle

  @param  peripheral Target peripheral clock wait on
  @retval void
*/
void clockWaitBusy(clock_peripheral_t peripheral)
{
  bcm2837_clock_t* clock = clockGetPeripheralClock(peripheral);

  assert(clock != NULL);

  while(clock->CTL.BUSY);

  RMB();
}

/**
  @brief  Configure the target peripheral clock source and divisor

  @param  peripheral Target peripheral clock to configure
  @param  source Clock source for clock generator
  @param  divi Integer portion of clock divider
  @param  divf Fractional portion of clock divider
  @retval void
*/
void clockConfigure(clock_peripheral_t peripheral, const clock_configuration_t* config)
{
  assert(config->divi > 0 && config->divi < 4096);
  assert(config->divf > 0 && config->divf < 4096);

  bcm2837_clock_t* clock = clockGetPeripheralClock(peripheral);

  assert(clock != NULL);

  // Disable clock generator
  WMB();
  clock->CTL.ENAB = 0;
  
  // Wait for idle
  while (clock->CTL.BUSY);
  RMB();

  // Don't configure if null
  if (config == NULL)
    return;

  // Configure source
  clock_control_t control = (clock_control_t) {0};
  control.PASSWD = CLOCK_MANAGER_PASSWORD;
  control.SRC = config->source;
  control.MASH = config->mash;
  control.FLIP = config->invert;

  // Configure divisor
  clock_divisor_t divisor = (clock_divisor_t) {0};
  divisor.PASSWD = CLOCK_MANAGER_PASSWORD;
  divisor.DIVI = config->divi;
  divisor.DIVF = config->divf;

  // Write to device
  clock->CTL = control;
  clock->DIV = divisor;
}