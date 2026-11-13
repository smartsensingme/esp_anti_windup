#include "esp_anti_windup.h"
#include "esp_attr.h"
#include <math.h>
#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#endif
#if !defined(ESP_PLATFORM) || CONFIG_ESP_ANTI_WINDUP_IRAM
#define AW_FAST IRAM_ATTR
#else
#define AW_FAST
#endif

/* Typed direct dispatch keeps participant ownership explicit and avoids
 * indirect user callbacks in the timing-sensitive execution path. */
esp_err_t AW_FAST esp_anti_windup_reset(esp_anti_windup_t *group) {
  if (!group || !group->initialized)
    return ESP_ERR_INVALID_STATE;
  esp_err_t first = ESP_OK;
  for (size_t i = 0; i < group->count; ++i) {
    const esp_anti_windup_participant_t *p = &group->participants[i];
    const esp_err_t err = p->kind == ESP_ANTI_WINDUP_PID
                              ? esp_pid_reset(p->controller.pid)
                              : esp_pr_reset(p->controller.pr);
    if (first == ESP_OK)
      first = err;
  }
  return first;
}

esp_err_t esp_anti_windup_init(esp_anti_windup_t *group,
                               const esp_anti_windup_config_t *config) {
  if (!group || !config || !config->participants || config->count == 0 ||
      config->count > ESP_ANTI_WINDUP_MAX_PARTICIPANTS ||
      !isfinite(config->minimum) || !isfinite(config->maximum) ||
      config->minimum >= config->maximum || !isfinite(config->dt_s) ||
      config->dt_s <= 0 || !isfinite(config->tracking_time_s) ||
      config->tracking_time_s < 0)
    return ESP_ERR_INVALID_ARG;
  esp_anti_windup_t next = {0};
  float sum = 0;
  for (size_t i = 0; i < config->count; ++i) {
    const esp_anti_windup_participant_t p = config->participants[i];
    if (!isfinite(p.weight) || p.weight < 0)
      return ESP_ERR_INVALID_ARG;
    const void *address;
    if (p.kind == ESP_ANTI_WINDUP_PID) {
      const esp_pid_t *pid = p.controller.pid;
      if (!pid || !pid->initialized || pid->tracking_pending ||
          pid->config.tracking_mode != ESP_PID_TRACKING_EXTERNAL ||
          pid->config.output_min != config->minimum ||
          pid->config.output_max != config->maximum ||
          (pid->config.maximum_dt_s > 0 &&
           config->dt_s >= pid->config.maximum_dt_s) ||
          (p.weight > 0 && pid->config.ki == 0))
        return ESP_ERR_INVALID_ARG;
      address = pid;
    } else if (p.kind == ESP_ANTI_WINDUP_PR) {
      const esp_pr_t *pr = p.controller.pr;
      if (!pr || !pr->model || !pr->model->prepared || pr->tracking_pending ||
          pr->model->tracking_mode != ESP_PR_TRACKING_EXTERNAL ||
          pr->model->output_min != config->minimum ||
          pr->model->output_max != config->maximum ||
          (p.weight > 0 && !pr->model->tracking_supported))
        return ESP_ERR_INVALID_ARG;
      address = pr;
    } else
      return ESP_ERR_INVALID_ARG;
    for (size_t j = 0; j < i; ++j) {
      const esp_anti_windup_participant_t *prev = &next.participants[j];
      const void *previous = prev->kind == ESP_ANTI_WINDUP_PID
                                 ? (const void *)prev->controller.pid
                                 : (const void *)prev->controller.pr;
      if (previous == address)
        return ESP_ERR_INVALID_ARG;
    }
    next.participants[i] = p;
    sum += p.weight;
    if (!isfinite(sum))
      return ESP_ERR_INVALID_ARG;
  }
  /* Expensive math and weight normalization occur only before the loop. */
  for (size_t i = 0; i < config->count; ++i)
    next.participants[i].weight =
        sum > 0 ? next.participants[i].weight / sum : 0;
  next.count = config->count;
  next.minimum = config->minimum;
  next.maximum = config->maximum;
  next.dt_s = config->dt_s;
  next.alpha = config->tracking_time_s > 0
                   ? -expm1f(-config->dt_s / config->tracking_time_s)
                   : 0;
  next.initialized = true;
  const esp_err_t err = esp_anti_windup_reset(&next);
  if (err != ESP_OK)
    return err;
  *group = next;
  return ESP_OK;
}

esp_err_t AW_FAST esp_anti_windup_update(esp_anti_windup_t *group,
                                         float reference, float measurement,
                                         esp_anti_windup_result_t *result) {
  if (!result)
    return ESP_ERR_INVALID_ARG;
  *result = (esp_anti_windup_result_t){0};
  if (!group || !group->initialized)
    return ESP_ERR_INVALID_STATE;
  esp_err_t err = ESP_ERR_INVALID_ARG;
  if (!isfinite(reference) || !isfinite(measurement))
    goto fail;
  esp_anti_windup_result_t next = {0};
  for (size_t i = 0; i < group->count; ++i) {
    const esp_anti_windup_participant_t *p = &group->participants[i];
    bool supported, pending;
    if (p->kind == ESP_ANTI_WINDUP_PID) {
      esp_pid_result_t r;
      err = esp_pid_update_result(p->controller.pid, reference, measurement,
                                  group->dt_s, &r);
      if (err != ESP_OK)
        goto fail;
      next.requests[i] = r.requested_output;
      supported = r.tracking_supported;
      pending = r.tracking_pending;
    } else {
      esp_pr_result_t r;
      err = esp_pr_update_result(p->controller.pr, reference, measurement, &r);
      if (err != ESP_OK)
        goto fail;
      next.requests[i] = r.requested_output;
      supported = r.tracking_supported;
      pending = r.tracking_pending;
    }
    if (!pending || (p->weight > 0 && !supported)) {
      err = ESP_ERR_INVALID_STATE;
      goto fail;
    }
    next.requested += next.requests[i];
  }
  /* Clamp the SUM only. Individually opposing requests can exceed rails. */
  next.applied = next.requested < group->minimum   ? group->minimum
                 : next.requested > group->maximum ? group->maximum
                                                   : next.requested;
  next.saturation_error = next.applied - next.requested;
  const float correction = group->alpha * next.saturation_error;
  if (!isfinite(next.requested) || !isfinite(next.saturation_error) ||
      !isfinite(correction)) {
    err = ESP_ERR_INVALID_ARG;
    goto fail;
  }
  next.saturated = next.applied != next.requested;
  for (size_t i = 0; i < group->count; ++i) {
    const esp_anti_windup_participant_t *p = &group->participants[i];
    const float share = p->weight * correction;
    err = p->kind == ESP_ANTI_WINDUP_PID
              ? esp_pid_apply_tracking(p->controller.pid, share)
              : esp_pr_apply_tracking(p->controller.pr, share);
    if (err != ESP_OK)
      goto fail;
  }
  *result = next;
  return ESP_OK;
fail:
  /* Never publish a partially updated actuator request; clear every state
   * including participants already corrected and those not yet visited. */
  esp_anti_windup_reset(group);
  return err;
}
