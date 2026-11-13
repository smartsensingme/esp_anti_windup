#include "anti_windup.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>

int main(void) {
  anti_windup_t c;
  assert(anti_windup_init(&c, -2, 3, 0.001f, 0.01f) == ANTI_WINDUP_OK);
  assert(fabsf(c.alpha - (1.0f - expf(-0.1f))) < 1e-7f);
  anti_windup_result_t r;
  assert(anti_windup_compute(&c, 8, &r) == ANTI_WINDUP_OK);
  assert(r.applied == 3 && r.error == -5 && r.saturated);
  assert(fabsf(r.correction + 5 * c.alpha) < 1e-6f);
  assert(anti_windup_compute(&c, -8, &r) == ANTI_WINDUP_OK);
  assert(r.applied == -2 && r.error == 6);
  assert(anti_windup_compute(&c, 3, &r) == ANTI_WINDUP_OK);
  assert(!r.saturated && r.correction == 0);
  /* Opposing requests are summed before limiting. */
  assert(anti_windup_compute(&c, 10.0f - 9.0f, &r) == ANTI_WINDUP_OK);
  assert(r.applied == 1 && !r.saturated);

  anti_windup_result_t saved = r;
  assert(anti_windup_compute(&c, NAN, &r) == ANTI_WINDUP_INVALID_ARGUMENT);
  assert(memcmp(&saved, &r, sizeof r) == 0);
  assert(anti_windup_compute(NULL, 0, &r) == ANTI_WINDUP_INVALID_ARGUMENT);
  assert(anti_windup_compute(&c, 0, NULL) == ANTI_WINDUP_INVALID_ARGUMENT);
  anti_windup_t old = c;
  assert(anti_windup_init(&c, 1, 1, 1, 1) == ANTI_WINDUP_INVALID_ARGUMENT);
  assert(anti_windup_init(&c, 0, INFINITY, 1, 1) ==
         ANTI_WINDUP_INVALID_ARGUMENT);
  assert(anti_windup_init(&c, -1, 1, 0, 1) == ANTI_WINDUP_INVALID_ARGUMENT);
  assert(anti_windup_init(&c, -1, 1, 1, -1) == ANTI_WINDUP_INVALID_ARGUMENT);
  assert(memcmp(&old, &c, sizeof c) == 0);
  assert(anti_windup_init(&c, -2, 3, 0.001f, 0) == ANTI_WINDUP_OK);
  assert(anti_windup_compute(&c, 8, &r) == ANTI_WINDUP_OK);
  assert(r.applied == 3 && r.correction == 0);
  /* Finite inputs may still overflow their difference. */
  assert(anti_windup_init(&c, FLT_MAX / 2, FLT_MAX, 1, 1) == ANTI_WINDUP_OK);
  saved = r;
  assert(anti_windup_compute(&c, -FLT_MAX, &r) == ANTI_WINDUP_INVALID_ARGUMENT);
  assert(memcmp(&saved, &r, sizeof r) == 0);
  /* Shared tracking bounds the total accumulator and recovers on reversal. */
  assert(anti_windup_init(&c, -1, 1, 0.001f, 0.01f) == ANTI_WINDUP_OK);
  float a = 0, b = 0;
  for (int i = 0; i < 2000; ++i) {
    const float e = i < 1000 ? 1 : -1;
    a += 0.004f * e;
    b += 0.002f * e;
    assert(anti_windup_compute(&c, a + b, &r) == ANTI_WINDUP_OK);
    a += 0.5f * r.correction;
    b += 0.5f * r.correction;
    assert(fabsf(a + b) < 1.1f);
  }
  assert(r.applied == -1);
  return 0;
}
