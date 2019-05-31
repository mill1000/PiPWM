#include <stddef.h>
#include <assert.h>
#include <time.h>

#include <bcm2837_gpio.h>

static bcm2837_gpio_t* gpio = NULL;

/**
  @brief  Initalize the GPIO object at the given base address

  @param  base Base address of GPIO peripheral
  @retval none
*/
void gpioInit(void* base)
{
  assert(base != NULL);
  assert(gpio == NULL);

  gpio = base;
}

/**
  @brief  Set the GPIO pin function

  @param  pin GPIO pin to set
  @param  function GPIO function to set
  @retval none
*/
static void gpioSetFunction(gpio_pin_t pin, gpio_function_t function)
{
  volatile gpio_function_select_x_t* functionSelect = &gpio->GPFSELx[pin / 10];

  switch (pin % 10)
  {
    case 0: functionSelect->FSELx0 = function; break;
    case 1: functionSelect->FSELx1 = function; break;
    case 2: functionSelect->FSELx2 = function; break;
    case 3: functionSelect->FSELx3 = function; break;
    case 4: functionSelect->FSELx4 = function; break;
    case 5: functionSelect->FSELx5 = function; break;
    case 6: functionSelect->FSELx6 = function; break;
    case 7: functionSelect->FSELx7 = function; break;
    case 8: functionSelect->FSELx8 = function; break;
    case 9: functionSelect->FSELx9 = function; break;
    default: assert(false); break;
  }
}

/**
  @brief  Configure the target GPIO pin

  @param  pin GPIO pin to configure
  @param  config GPIO configuration to set
  @retval void
*/
void gpioConfigure(gpio_pin_t pin, const gpio_configuration_t* config)
{
  assert(gpio != NULL);
  assert(config != NULL);
  assert(pin < GPIO_PIN_COUNT);

  WMB();

  gpioSetFunction(pin, config->function);

  // Determine which 32-bit "bank" the pin is in
  uint8_t bank = pin / 32;

  // Construct a mask for the pin number in it's bank
  uint32_t mask = 1 << (pin % 32);

  // Clear all event registers
  gpio->GPRENx[bank].REN &= ~mask;
  gpio->GPFENx[bank].FEN &= ~mask;
  gpio->GPHENx[bank].HEN &= ~mask;
  gpio->GPLENx[bank].LEN &= ~mask;
  gpio->GPARENx[bank].AREN &= ~mask;
  gpio->GPAFENx[bank].AFEN &= ~mask;

  switch (config->eventDetect)
  {
    case gpio_event_detect_none:
      // Do nothing
      break;

    case gpio_event_detect_rising_edge:
      gpio->GPRENx[bank].REN |= mask;
      break;

    case gpio_event_detect_falling_edge:
      gpio->GPFENx[bank].FEN |= mask;
      break;

    case gpio_event_detect_any_edge:
      gpio->GPRENx[bank].REN |= mask;
      gpio->GPFENx[bank].FEN |= mask;
      break;

    case gpio_event_detect_high_level:
      gpio->GPHENx[bank].HEN |= mask;
      break;

    case gpio_event_detect_low_level:
      gpio->GPLENx[bank].LEN |= mask;
      break;

    case gpio_event_detect_rising_edge_async:
      gpio->GPARENx[bank].AREN |= mask;
      break;

    case gpio_event_detect_falling_edge_async:
      gpio->GPAFENx[bank].AFEN |= mask;
      break;

    default:
    break;
  }

  if (config->pull != gpio_pull_no_change)
  {
    // Set the mode bits
    gpio->GPPUD.PUD = config->pull;

    struct timespec delay;
    delay.tv_sec = 0;
    delay.tv_nsec = 10e3;

    // Wait at least 150 cycles. Total guess.
    nanosleep(&delay, NULL);

    // Set the clock bit for target pin
    gpio->GPPUDCLKx[bank].PUDCLK |= mask;

    // Wait 150 more cycles
    nanosleep(&delay, NULL);

    // Clear clock
    gpio->GPPUDCLKx[bank].PUDCLK &= ~mask;
  }

  RMB();
}