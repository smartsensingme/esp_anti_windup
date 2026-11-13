#include "anti_windup.h"
#include <math.h>
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif

/* Optional platform placement; portable builds have no platform includes. */
#ifndef ANTI_WINDUP_FAST_ATTR
#if defined(ESP_PLATFORM) && defined(CONFIG_ANTI_WINDUP_IRAM) &&               \
    CONFIG_ANTI_WINDUP_IRAM
#include "esp_attr.h"
#define ANTI_WINDUP_FAST_ATTR IRAM_ATTR
#else
#define ANTI_WINDUP_FAST_ATTR
#endif
#endif

anti_windup_status_t anti_windup_init(anti_windup_t *config, float minimum,
                                      float maximum, float dt_s,
                                      float tracking_time_s) {
  /* Validate first so rejected configuration never changes the destination. */
  if (!config || !isfinite(minimum) || !isfinite(maximum) ||
      minimum >= maximum || !isfinite(dt_s) || dt_s <= 0.0f ||
      !isfinite(tracking_time_s) || tracking_time_s < 0.0f)
    return ANTI_WINDUP_INVALID_ARGUMENT;

  /* This expensive invariant is evaluated only during initialization. */
  const float alpha =
      tracking_time_s > 0.0f ? -expm1f(-dt_s / tracking_time_s) : 0.0f;
  *config = (anti_windup_t){minimum, maximum, alpha};
  return ANTI_WINDUP_OK;
}

ANTI_WINDUP_FAST_ATTR anti_windup_status_t
anti_windup_compute(const anti_windup_t *config, float requested,
                    anti_windup_result_t *result) {
  /* Validate the public configuration too, to reject uninitialized/corrupt
   * values. This does not permit a concurrent writer. */
  if (!config || !result || !isfinite(requested) ||
      !isfinite(config->minimum) || !isfinite(config->maximum) ||
      config->minimum >= config->maximum || !isfinite(config->alpha) ||
      config->alpha < 0.0f || config->alpha > 1.0f)
    return ANTI_WINDUP_INVALID_ARGUMENT;

  const float applied = requested < config->minimum   ? config->minimum
                        : requested > config->maximum ? config->maximum
                                                      : requested;
  const float error = applied - requested;
  const float correction = config->alpha * error;
  if (!isfinite(error) || !isfinite(correction))
    return ANTI_WINDUP_INVALID_ARGUMENT;

  /* Commit an entire snapshot only after successful arithmetic. */
  *result = (anti_windup_result_t){requested, applied, error, correction,
                                   requested != applied};
  return ANTI_WINDUP_OK;
}
