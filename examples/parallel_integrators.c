#include "anti_windup.h"
#include <stdio.h>

/* Two controller accumulators sharing one bounded actuator. This is a
 * numerical demonstration, not a plant model or controller-tuning example. */
int main(void) {
  anti_windup_t tracking;
  if (anti_windup_init(&tracking, -1.0f, 1.0f, 0.001f, 0.02f) != ANTI_WINDUP_OK)
    return 1;
  float first = 0.0f, second = 0.0f;
  for (int n = 0; n < 2000; ++n) {
    const float error = n < 1000 ? 1.0f : -1.0f;
    first += 0.001f * 4.0f * error;
    second += 0.001f * 2.0f * error;
    anti_windup_result_t result;
    if (anti_windup_compute(&tracking, first + second, &result) !=
        ANTI_WINDUP_OK)
      return 1;
    /* Weights sum to one; each integrator receives its state increment. */
    first += 0.5f * result.correction;
    second += 0.5f * result.correction;
    if (n % 250 == 0)
      printf("step=%d requested=%f applied=%f correction=%f\n", n,
             (double)result.requested, (double)result.applied,
             (double)result.correction);
  }
  return 0;
}
