#pragma once
#include "esp_pid.h"
#include "esp_pr.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Fixed storage bound. Work is bounded by registered count (1..8). */
#define ESP_ANTI_WINDUP_MAX_PARTICIPANTS 8U
/** Supported controller kinds; dispatch is direct, without callbacks. */
typedef enum { ESP_ANTI_WINDUP_PID, ESP_ANTI_WINDUP_PR } esp_anti_windup_kind_t;
/** Controller reference. No ownership transfer; instance and PR model must
 * outlive the coordinator and remain at stable addresses. */
typedef struct {
  esp_anti_windup_kind_t kind; /**< Selects the valid union member. */
  union {
    esp_pid_t *pid;
    esp_pr_t *pr;
  } controller;
  float weight; /**< Nonnegative tracking weight; zero permits pure P. */
} esp_anti_windup_participant_t;
/** Initialization parameters; array is copied, controllers are not copied. */
typedef struct {
  float minimum; /**< Finite lower command bound, strictly below maximum. */
  float maximum; /**< Finite upper bound, same units as every output. */
  float dt_s; /**< Fixed positive period used by every PID. PR must be tuned to
                 it. */
  float tracking_time_s; /**< Nonnegative; zero disables correction only. */
  const esp_anti_windup_participant_t
      *participants; /**< Borrowed during init. */
  size_t count;      /**< 1..ESP_ANTI_WINDUP_MAX_PARTICIPANTS, no duplicate
                        instances. */
} esp_anti_windup_config_t;
/** Per-step diagnostics, in common actuator-command units. */
typedef struct {
  float requested;        /**< Sum of unsaturated requests. */
  float applied;          /**< Single clamp of requested. */
  float saturation_error; /**< applied-requested. */
  bool saturated;         /**< True when sum was limited. */
  float requests[ESP_ANTI_WINDUP_MAX_PARTICIPANTS]; /**< Registration order. */
} esp_anti_windup_result_t;
/** Caller-owned storage. Initialize before use; fields are read-only to users.
 * Do not copy/move registered controller storage or mutate configuration while
 * operating. One serialized task owns group and all its participants. */
typedef struct {
  esp_anti_windup_participant_t participants[ESP_ANTI_WINDUP_MAX_PARTICIPANTS];
  size_t count;
  float minimum, maximum, dt_s, alpha;
  bool initialized;
} esp_anti_windup_t;

/** Public setup, task context before acquisition; calls participant resets.
 * Validates external-tracking mode, matching local/output bounds, finite
 * weights and positive-period PID timing. PR coefficients' period is caller's
 * responsibility (the model does not retain sample rate). Nonzero weights on
 * pure-P PR or PID with Ki=0 are rejected. Normalize weights once at init;
 * all-zero weights permit P-only limiting. No allocation. Invalid arguments
 * leave group unchanged. Participant reset failure leaves group unchanged but
 * may have reset preceding participants. Reject preexisting pending tracking.
 */
esp_err_t esp_anti_windup_init(esp_anti_windup_t *group,
                               const esp_anti_windup_config_t *config);
/** Public reset, also called internally after a failed update. Resets ALL
 * registered participants, cancelling pending tracking; attempts all even if
 * one fails and returns the first error. Configuration remains prepared. */
esp_err_t esp_anti_windup_reset(esp_anti_windup_t *group);
/** Public fixed-period step: update every participant with the same reference
 * and measurement, sum requests, clamp once, distribute correction through
 * public tracking APIs (including zero). No allocation, locks, logging or I/O.
 * On any update/numeric/tracking failure, reset all participants and return the
 * originating error; reset failure is available by an explicit reset call.
 * With a valid result pointer, failure clears it to zero; never actuate a
 * failed result. Null result returns INVALID_ARG without advancing state.
 * Uninitialized/null group returns INVALID_STATE. Output before DAC
 * quantization. Calls update_result/apply_tracking APIs; optional IRAM includes
 * reset path. Data, models, stack and callees need separate cache-safety audit.
 */
esp_err_t esp_anti_windup_update(esp_anti_windup_t *group, float reference,
                                 float measurement,
                                 esp_anti_windup_result_t *result);
#ifdef __cplusplus
}
#endif
