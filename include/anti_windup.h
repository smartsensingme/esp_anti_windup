#ifndef ANTI_WINDUP_H
#define ANTI_WINDUP_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Portable status. No platform headers or error types are required. */
typedef enum {
  ANTI_WINDUP_OK = 0,
  ANTI_WINDUP_INVALID_ARGUMENT
} anti_windup_status_t;

/** Prepared, immutable configuration. Treat fields as read-only after init.
 * All signals and limits use the same actuator-command units. */
typedef struct {
  float minimum; /**< Finite lower command bound, strictly below maximum. */
  float maximum; /**< Finite upper command bound. */
  float alpha;   /**< Relaxation factor in [0,1], prepared from dt and time. */
} anti_windup_t;

/** Per-step result, independent of controller implementation. */
typedef struct {
  float requested;  /**< Sum before limiting, in actuator-command units. */
  float applied;    /**< Command clamped to [minimum, maximum]. */
  float error;      /**< applied - requested, same units. */
  float correction; /**< alpha * error; complete state increment, same units. */
  bool saturated;   /**< True exactly when the command was limited. */
} anti_windup_result_t;

/** Public initialization; not called internally. Prepare before periodic work.
 * dt_s must be finite and positive; tracking_time_s finite and nonnegative.
 * Zero tracking time disables correction, but retains command limiting.
 * Otherwise alpha = 1-exp(-dt_s/tracking_time_s). No allocation or locks.
 * On failure, destination is unchanged (including for nonfinite limits).
 * Configuration belongs to caller and must outlive its use; serialize writes.
 * Initialization uses libm and is not placed in IRAM by this component.
 */
anti_windup_status_t anti_windup_init(anti_windup_t *config, float minimum,
                                      float maximum, float dt_s,
                                      float tracking_time_s);

/** Public per-step calculation; not called internally. No controller state is
 * owned or changed. Pass the SUM of unsaturated requests in common output
 * units. Caller distributes correction among dynamic states and applies it
 * once, even when zero, according to each controller's external-tracking API.
 * No allocation, locks, callbacks or transcendental math. Constant work.
 * Config must be initialized and immutable during this call. Concurrent calls
 * may share config with distinct result storage. Result is caller-owned;
 * failures leave it unchanged. Null/invalid config, nonfinite request or
 * arithmetic overflow returns INVALID_ARGUMENT. No actuator I/O is performed.
 * IRAM placement is optional at build time; see README for full-path limits.
 */
anti_windup_status_t anti_windup_compute(const anti_windup_t *config,
                                         float requested,
                                         anti_windup_result_t *result);

#ifdef __cplusplus
}
#endif
#endif
