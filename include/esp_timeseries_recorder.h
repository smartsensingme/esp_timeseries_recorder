#ifndef ESP_TIMESERIES_RECORDER_H_
#define ESP_TIMESERIES_RECORDER_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Reserved sample code used when a floating-point input is not finite. */
#define ESP_TIMESERIES_INVALID_I16 INT16_MIN

/** Recorder states. Only a full capture may be read or cleared. */
typedef enum {
  ESP_TIMESERIES_STATE_UNINITIALIZED = 0,
  ESP_TIMESERIES_STATE_EMPTY,
  ESP_TIMESERIES_STATE_ARMED,
  ESP_TIMESERIES_STATE_CAPTURING,
  ESP_TIMESERIES_STATE_FULL,
} esp_timeseries_state_t;

/** Metadata needed to reconstruct one signed 16-bit channel. */
typedef struct {
  const char *name;
  const char *unit;
  float scale;
  float offset;
} esp_timeseries_channel_t;

/** Static recorder configuration. Descriptor strings must have static life. */
typedef struct {
  uint32_t producer_rate_hz;
  size_t channel_count;
  const esp_timeseries_channel_t *channels;
} esp_timeseries_config_t;

/** Coherent status copy safe to inspect from a consumer task. */
typedef struct {
  esp_timeseries_state_t state;
  uint32_t producer_rate_hz;
  uint32_t sample_rate_hz;
  uint32_t sample_divider;
  uint32_t capture_id;
  size_t channel_count;
  size_t buffer_bytes;
  size_t record_bytes;
  size_t sample_capacity;
  size_t sample_count;
  int64_t start_time_us;
  uintptr_t buffer_begin;
  uintptr_t buffer_end;
} esp_timeseries_status_t;

/** Immutable view valid while the recorder remains FULL. */
typedef struct {
  const int16_t *samples;
  size_t sample_count;
  size_t sample_capacity;
  size_t channel_count;
  size_t payload_bytes;
  uint32_t sample_rate_hz;
  uint32_t capture_id;
  int64_t start_time_us;
  const esp_timeseries_channel_t *channels;
  const uint32_t *saturation_counts;
  const uint32_t *invalid_counts;
} esp_timeseries_capture_t;

/** Initialize the singleton recorder and its statically allocated buffer. */
esp_err_t esp_timeseries_init(const esp_timeseries_config_t *config);

/**
 * Arm the next producer call as the first sample of a new capture.
 *
 * sample_rate_hz must be nonzero, no greater than producer_rate_hz, and an
 * exact integer divisor of producer_rate_hz. Only an EMPTY recorder can arm.
 */
esp_err_t esp_timeseries_arm(uint32_t sample_rate_hz);

/**
 * Quantize and conditionally store one producer-rate sample.
 *
 * Values use the real units declared by each descriptor. The function returns
 * true only when this producer call stored a record. It is single-producer and
 * bounded: no allocation, logging, transport, or blocking is performed.
 */
bool esp_timeseries_record_f32(const float *values, int64_t timestamp_us);

/** Store already-quantized int16 values with the same decimation behavior. */
bool esp_timeseries_record_i16(const int16_t *values, int64_t timestamp_us);

/** Fast, lock-free state query suitable for a producer hot path. */
esp_timeseries_state_t esp_timeseries_get_state(void);

/** Copy recorder state. Status may be approximate while capture is active. */
esp_err_t esp_timeseries_get_status(esp_timeseries_status_t *status);

/** Obtain a stable capture view. Returns ESP_ERR_INVALID_STATE unless FULL. */
esp_err_t esp_timeseries_get_capture(esp_timeseries_capture_t *capture);

/** Release a FULL capture. Data remain untouched but become invalid to read. */
esp_err_t esp_timeseries_clear(void);

/** Human-readable state name for status protocols and diagnostics. */
const char *esp_timeseries_state_name(esp_timeseries_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* ESP_TIMESERIES_RECORDER_H_ */
