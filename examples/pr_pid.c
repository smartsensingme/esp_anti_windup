/* ESP-IDF example: compile as a main component depending on esp_anti_windup.
 * Numerical exercise only: no ADC, DAC, plant or periodic acquisition. */
#include "esp_anti_windup.h"
#include "esp_log.h"

void app_main(void) {
  static esp_pid_t integral;
  static esp_pr_t pr;
  static esp_pr_model_t model;
  static esp_anti_windup_t group;
  const esp_pr_resonator_config_t branch = {
      .frequency_hz = 60, .kr = 2, .damping_hz = 5};
  const esp_pr_config_t pc = {.sample_rate_hz = 14400,
                              .kp = 0.5f,
                              .output_min = -2,
                              .output_max = 2,
                              .resonators = &branch,
                              .resonator_count = 1,
                              .tracking_mode = ESP_PR_TRACKING_EXTERNAL};
  const esp_pid_config_t ic = {.ki = 1,
                               .output_min = -2,
                               .output_max = 2,
                               .tracking_mode = ESP_PID_TRACKING_EXTERNAL};
  ESP_ERROR_CHECK(esp_pr_prepare(&model, &pc));
  ESP_ERROR_CHECK(esp_pr_init(&pr, &model));
  ESP_ERROR_CHECK(esp_pid_init(&integral, &ic));
  const esp_anti_windup_participant_t participants[] = {
      {.kind = ESP_ANTI_WINDUP_PR, .controller.pr = &pr, .weight = 1},
      {.kind = ESP_ANTI_WINDUP_PID, .controller.pid = &integral, .weight = 1},
  };
  const esp_anti_windup_config_t config = {.minimum = -2,
                                           .maximum = 2,
                                           .dt_s = 1.0f / 14400,
                                           .tracking_time_s = 0.01f,
                                           .participants = participants,
                                           .count = 2};
  ESP_ERROR_CHECK(esp_anti_windup_init(&group, &config));
  esp_anti_windup_result_t result;
  for (int i = 0; i < 1000; ++i)
    ESP_ERROR_CHECK(esp_anti_windup_update(&group, 0.1f, 0, &result));
  ESP_LOGI("example", "requested=%f applied=%f", (double)result.requested,
           (double)result.applied);
}
