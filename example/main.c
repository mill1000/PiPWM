#include <unistd.h>
#include <float.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>

#include "log.h"
#include "pipwm.h"

#define TAG "MAIN"

/**
  @brief  Callback function for POSIX signals

  @param  signal Received POSIX signal
  @retval none
*/
static void signalHandler(int32_t signal)
{
  LOGW(TAG, "Received signal %s (%d).", sys_siglist[signal], signal);

  // Safetly shutdown the PWM system
  piPwm_shutdown();

  // Termiante
  exit(EXIT_SUCCESS);
}

/**
  @brief  Reigster a handler for all POSIX signals 
          that would cause termination

  @param  none
  @retval none
*/
void registerSignalHandler()
{
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));

  sa.sa_handler = &signalHandler;

  // Register fatal signal handlers
  sigaction(SIGHUP, &sa, NULL);
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGQUIT, &sa, NULL);
  sigaction(SIGILL, &sa, NULL);
  sigaction(SIGABRT, &sa, NULL);
  sigaction(SIGFPE, &sa, NULL);
  sigaction(SIGSEGV, &sa, NULL);
  sigaction(SIGPIPE, &sa, NULL);
  sigaction(SIGALRM, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sigaction(SIGBUS, &sa, NULL);
}

/**
  @brief  Main entry point

  @param  argc
  @param  argv
  @retval none
*/
int main (int argc, char* argv[])
{
  // Register signal handlers
  registerSignalHandler();

  // Increase logging level to debug
  logSetLevel(LOG_LEVEL_DEBUG);
    
  // Initalize PWM system with requested resolution fo 10 us
  piPwm_initialize(dma_channel_13, 10e-6);

  gpio_pin_mask_t pins = 1 << 12 | 1 << 16;

  // Initalize PWM channel at 100 Hz on pins 12 & 16
  pipwm_channel_t* channel = piPwm_initalizeChannel(dma_channel_14, pins, 100);
  if (channel == NULL)
    LOGF(TAG, "Failed to initalize PiPWM channel");

  // Enable the channel
  piPwm_enableChannel(channel, true);
  
  piPwm_setDutyCycle(channel, pins, .25);
  sleep(2);

  piPwm_setPulseWidth(channel, 1 << 12, 1.52e-3);
  sleep(2);

  piPwm_setDutyCycle(channel, 1 << 16, .5);
  sleep(2);

  while(1)
    sleep(1);
    
  return 0;
}