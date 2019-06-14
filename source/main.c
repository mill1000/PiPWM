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
  
  double frequency_Hz = 100;
  double tCycle_s = 1 / frequency_Hz;
  double tStep_s = 10e-6;

  uint32_t steps = round(tCycle_s / tStep_s);

  LOGI(TAG, "Delta T: %g us, Clock: %g Hz, Steps: %d", tStep_s * 1e6, frequency_Hz, steps);

  double srcClock_Hz = 19.2e6;

  double divider = srcClock_Hz * tStep_s;
  LOGI(TAG, "Target divisor: %g", divider);

  //uint32_t minDivi = 0;
  //uint32_t minRange = 0;
  //double minError = DBL_MAX;
  //double tStepCalc_s = 0.0;
  //uint32_t range = 2;
  //uint32_t divi = divider / 2;
  //while (divi > 1)
  //{
  //  divi = divider / range;
  //  tStepCalc_s = (divi * range) / srcClock_Hz;
  //  
  //  double error = fabs(tStep_s - tStepCalc_s);
  //  
  //  LOGI(TAG, "DIVI: %d, Range: %d, Error: %g", divi, range, error);
  //  
  //  if (error < minError)
  //  {
  //    minError = error;
  //    minRange = range;
  //    minDivi = divi;
  //  }
  //
  //  if (error < 1e-9)
  //    break;
  //
  //  range++;
  //}
  //LOGI(TAG, "Best DIVI: %d, Best Range: %d, Min error: %g", minDivi, minRange, minError);
  //tStepCalc_s = srcClock_Hz / (minDivi * minRange);
  //LOGI(TAG, "Clk Divider: %d, Range: %d, Actual delta T: %g us", divi, range, 1e6/tStepCalc_s);

  double integral = 0;
  double frac = modf(divider / 2, &integral);
  
  piPwm_initialize(dma_channel_13, integral, frac * 4096, 2);
 
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