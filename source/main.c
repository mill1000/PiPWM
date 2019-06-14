#include <unistd.h>
#include <float.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>

#include "log.h"
#include "pipwm.h"

#define TAG "MAIN"

void signalHandler(int signal)
{
  LOGW(TAG, "Received signal %s (%d).", sys_siglist[signal], signal);

  piPwm_shutdown();

  exit(EXIT_SUCCESS);
}

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

int main()
{
  registerSignalHandler();

  logSetLevel(LOG_LEVEL_DEBUG);
    
  double tStep_s = 33e-6;

  piPwm_initialize(dma_channel_13, tStep_s);
 
  gpio_pin_mask_t mask = 1 << 12 | 1 << 16;

  pipwm_channel_t* channel = piPwm_initalizeChannel(dma_channel_14, mask, 100);
  if (channel == NULL)
    LOGF(TAG, "Failed to initalize PiPWM channel");

  piPwm_enableChannel(channel, true);

  piPwm_setDutyCycle(channel, mask, .25);
  sleep(2);

  piPwm_setPulseWidth(channel, 1 << 12, 1.52e-3);
  sleep(2);

  piPwm_setDutyCycle(channel, 1 << 16, .5);
  sleep(2);

  while(1)
    sleep(1);
    
  return 0;
}