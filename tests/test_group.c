#include "esp_anti_windup.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

static void init_pid(esp_pid_t *pid, float ki) {
  const esp_pid_config_t c = {.ki = ki,
                              .output_min = -2,
                              .output_max = 2,
                              .tracking_mode = ESP_PID_TRACKING_EXTERNAL};
  assert(esp_pid_init(pid, &c) == ESP_OK);
}
int main(void) {
  esp_pid_t a, b;
  init_pid(&a, 2);
  init_pid(&b, 1);
  esp_anti_windup_participant_t p[] = {
      {.kind = ESP_ANTI_WINDUP_PID, .controller.pid = &a, .weight = 1},
      {.kind = ESP_ANTI_WINDUP_PID, .controller.pid = &b, .weight = 3},
  };
  esp_anti_windup_config_t c = {.minimum = -2,
                                .maximum = 2,
                                .dt_s = 0.001f,
                                .tracking_time_s = 0.01f,
                                .participants = p,
                                .count = 2};
  esp_anti_windup_t g;
  assert(esp_anti_windup_init(&g, &c) == ESP_OK);
  assert(g.participants[0].weight == 0.25f &&
         g.participants[1].weight == 0.75f);
  a.state.integral_term = 1.5f;
  b.state.integral_term = 1.5f;
  esp_anti_windup_result_t r;
  assert(esp_anti_windup_update(&g, 0, 0, &r) == ESP_OK);
  assert(r.requested == 3 && r.applied == 2 && r.saturated);
  assert(fabsf(a.state.integral_term - (1.5f - 0.25f * g.alpha)) < 1e-6f);
  assert(fabsf(b.state.integral_term - (1.5f - 0.75f * g.alpha)) < 1e-6f);
  assert(!a.tracking_pending && !b.tracking_pending);
  a.state.integral_term = 5;
  b.state.integral_term = -4;
  assert(esp_anti_windup_update(&g, 0, 0, &r) == ESP_OK);
  assert(r.applied == 1 && !r.saturated);
  assert(a.state.integral_term == 5 && b.state.integral_term == -4);

  /* Rejected configuration leaves group intact, including controller states. */
  esp_anti_windup_t old = g;
  p[1].controller.pid = &a;
  assert(esp_anti_windup_init(&g, &c) == ESP_ERR_INVALID_ARG);
  assert(memcmp(&old, &g, sizeof g) == 0 && a.state.integral_term == 5);
  p[1].controller.pid = &b;
  p[1].weight = NAN;
  assert(esp_anti_windup_init(&g, &c) == ESP_ERR_INVALID_ARG);
  p[1].weight = 1;
  c.count = ESP_ANTI_WINDUP_MAX_PARTICIPANTS + 1;
  assert(esp_anti_windup_init(&g, &c) == ESP_ERR_INVALID_ARG);
  c.count = 2;
  /* Failure after one update cancels all pending steps and publishes zero. */
  b.state.integral_term = NAN;
  assert(esp_anti_windup_update(&g, 1, 0, &r) != ESP_OK);
  assert(r.applied == 0 && a.state.integral_term == 0 &&
         b.state.integral_term == 0);
  assert(!a.tracking_pending && !b.tracking_pending);
  assert(esp_anti_windup_update(&g, 0, 0, &r) == ESP_OK);
  assert(esp_anti_windup_update(&g, NAN, 0, &r) != ESP_OK && r.applied == 0);
  assert(esp_anti_windup_update(&g, 1, 0, NULL) == ESP_ERR_INVALID_ARG);
  assert(!a.tracking_pending);
  c.tracking_time_s = 0;
  assert(esp_anti_windup_init(&g, &c) == ESP_OK);
  a.state.integral_term = 3;
  assert(esp_anti_windup_update(&g, 0, 0, &r) == ESP_OK);
  assert(r.applied == 2 && a.state.integral_term == 3);
  /* Pure proportional participants limit with no state correction. */
  init_pid(&a, 0);
  p[0].weight = 0;
  c.count = 1;
  a.config.kp = 10;
  assert(esp_anti_windup_init(&g, &c) == ESP_OK);
  assert(esp_anti_windup_update(&g, 1, 0, &r) == ESP_OK && r.applied == 2);
  p[0].weight = 1;
  assert(esp_anti_windup_init(&g, &c) == ESP_ERR_INVALID_ARG);
  init_pid(&a, 2);
  a.config.tracking_mode = ESP_PID_TRACKING_INTERNAL;
  assert(esp_anti_windup_init(&g, &c) == ESP_ERR_INVALID_ARG);
  /* Shared saturation and reversal remain bounded for unequal weights. */
  init_pid(&a, 2);
  init_pid(&b, 1);
  c.count = 2;
  c.tracking_time_s = 0.01f;
  assert(esp_anti_windup_init(&g, &c) == ESP_OK);
  for (int i = 0; i < 10000; ++i) {
    assert(esp_anti_windup_update(&g, i < 5000 ? 5 : -5, 0, &r) == ESP_OK);
    assert(fabsf(r.applied) <= 2 &&
           fabsf(a.state.integral_term + b.state.integral_term) < 2.2f);
  }
  assert(r.applied == -2);
  return 0;
}
