#include "esp_timeseries_recorder.h"

#include "sdkconfig.h"
#include <limits.h>
#include <math.h>
#include <stdatomic.h>
#include <string.h>

#define RECORDER_BUFFER_BYTES                                                  \
  ((size_t)CONFIG_ESP_TIMESERIES_RECORDER_BUFFER_KIB * 1024U)
#define RECORDER_STATE_CONFIGURING (-1)

_Static_assert(RECORDER_BUFFER_BYTES >= sizeof(int16_t),
               "Recorder buffer must hold at least one int16 value");

/* A singleton keeps the large, compile-time-sized DRAM reservation reusable. */
static int16_t recorder_buffer[RECORDER_BUFFER_BYTES / sizeof(int16_t)];
static uint32_t saturation_counts[CONFIG_ESP_TIMESERIES_RECORDER_MAX_CHANNELS];
static uint32_t invalid_counts[CONFIG_ESP_TIMESERIES_RECORDER_MAX_CHANNELS];
static esp_timeseries_config_t recorder_config;
static atomic_int recorder_state = ESP_TIMESERIES_STATE_UNINITIALIZED;
static atomic_size_t recorder_sample_count;
static uint32_t recorder_sample_rate_hz;
static uint32_t recorder_sample_divider;
static uint32_t recorder_capture_id;
static uint32_t producer_calls_until_sample;
static int64_t recorder_start_time_us;

/**
 * @brief Calculate the byte width of one interleaved record.
 *
 * Internal helper called by sample_capacity(), esp_timeseries_record_i16(),
 * esp_timeseries_get_status(), and esp_timeseries_get_capture().
 */
static size_t record_bytes(void) {
  return recorder_config.channel_count * sizeof(int16_t);
}

/**
 * @brief Calculate how many complete records fit in the static buffer.
 *
 * Internal helper used during initialization, producer bounds checking, status
 * reporting, capture publication, and FULL-state detection.
 */
static size_t sample_capacity(void) {
  size_t bytes = record_bytes();
  return bytes > 0U ? sizeof(recorder_buffer) / bytes : 0U;
}

/**
 * @brief Validate and publish the application-wide recorder configuration.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
esp_err_t esp_timeseries_init(const esp_timeseries_config_t *config) {
  /* Validate top-level limits before accessing channel descriptors. */
  if (config == NULL || config->channels == NULL ||
      config->producer_rate_hz == 0U || config->channel_count == 0U ||
      config->channel_count > CONFIG_ESP_TIMESERIES_RECORDER_MAX_CHANNELS) {
    return ESP_ERR_INVALID_ARG;
  }
  if (atomic_load_explicit(&recorder_state, memory_order_acquire) !=
      ESP_TIMESERIES_STATE_UNINITIALIZED) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Validate the linear transform and strings required for every channel. */
  for (size_t channel = 0; channel < config->channel_count; channel++) {
    if (config->channels[channel].name == NULL ||
        config->channels[channel].unit == NULL ||
        !isfinite(config->channels[channel].scale) ||
        config->channels[channel].scale <= 0.0f ||
        !isfinite(config->channels[channel].offset)) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  /* Shallow-copy persistent metadata and verify the static buffer geometry. */
  recorder_config = *config;
  if (sample_capacity() == 0U) {
    recorder_config = (esp_timeseries_config_t){0};
    return ESP_ERR_NO_MEM;
  }

  /* Reset capture metadata before making EMPTY visible to other tasks. */
  memset(saturation_counts, 0, sizeof(saturation_counts));
  memset(invalid_counts, 0, sizeof(invalid_counts));
  atomic_store_explicit(&recorder_sample_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&recorder_state, ESP_TIMESERIES_STATE_EMPTY,
                        memory_order_release);
  return ESP_OK;
}

/**
 * @brief Configure the decimator and atomically arm a new capture.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
esp_err_t esp_timeseries_arm(uint32_t sample_rate_hz) {
  /* Exact integer division guarantees uniform sample spacing. */
  if (sample_rate_hz == 0U ||
      sample_rate_hz > recorder_config.producer_rate_hz ||
      recorder_config.producer_rate_hz % sample_rate_hz != 0U) {
    return ESP_ERR_INVALID_ARG;
  }

  /* Claim EMPTY so a concurrent consumer cannot arm the same buffer twice. */
  int expected = ESP_TIMESERIES_STATE_EMPTY;
  if (!atomic_compare_exchange_strong_explicit(
          &recorder_state, &expected, RECORDER_STATE_CONFIGURING,
          memory_order_acq_rel, memory_order_acquire)) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Prepare all per-capture state while the private sentinel hides it. */
  recorder_sample_rate_hz = sample_rate_hz;
  recorder_sample_divider = recorder_config.producer_rate_hz / sample_rate_hz;
  recorder_capture_id++;
  producer_calls_until_sample = 0U;
  recorder_start_time_us = 0;
  memset(saturation_counts, 0, sizeof(saturation_counts));
  memset(invalid_counts, 0, sizeof(invalid_counts));
  atomic_store_explicit(&recorder_sample_count, 0U, memory_order_relaxed);

  /* Publish the fully configured capture to the producer. */
  atomic_store_explicit(&recorder_state, ESP_TIMESERIES_STATE_ARMED,
                        memory_order_release);
  return ESP_OK;
}

/**
 * @brief Decide whether the current producer call must store a record.
 *
 * Internal hot-path helper called only by esp_timeseries_record_f32() and
 * esp_timeseries_record_i16(). It starts an ARMED capture, advances the integer
 * decimator, and returns the next unpublished buffer index.
 */
static bool prepare_record(int64_t timestamp_us, size_t *sample_index) {
  /* The first call after ARM defines the capture timestamp and starts it. */
  esp_timeseries_state_t state = (esp_timeseries_state_t)atomic_load_explicit(
      &recorder_state, memory_order_acquire);
  if (state == ESP_TIMESERIES_STATE_ARMED) {
    recorder_start_time_us = timestamp_us;
    atomic_store_explicit(&recorder_state, ESP_TIMESERIES_STATE_CAPTURING,
                          memory_order_release);
    state = ESP_TIMESERIES_STATE_CAPTURING;
  }
  if (state != ESP_TIMESERIES_STATE_CAPTURING) {
    return false;
  }

  /* Skip producer calls until the next exact divider boundary. */
  if (producer_calls_until_sample > 0U) {
    producer_calls_until_sample--;
    return false;
  }
  producer_calls_until_sample = recorder_sample_divider - 1U;

  /* The single producer owns this unpublished destination record. */
  *sample_index =
      atomic_load_explicit(&recorder_sample_count, memory_order_relaxed);
  return *sample_index < sample_capacity();
}

/**
 * @brief Publish one completed record and transition to FULL when necessary.
 *
 * Internal hot-path helper called only by esp_timeseries_record_f32() and
 * esp_timeseries_record_i16(), after every channel has been written.
 */
static void finish_record(size_t sample_index) {
  /* Release publication prevents consumers from observing a partial record. */
  size_t count = sample_index + 1U;
  atomic_store_explicit(&recorder_sample_count, count, memory_order_release);

  /* FULL freezes the payload until an explicit consumer clear. */
  if (count >= sample_capacity()) {
    atomic_store_explicit(&recorder_state, ESP_TIMESERIES_STATE_FULL,
                          memory_order_release);
  }
}

/**
 * @brief Encode physical values and publish a decimated record.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
bool esp_timeseries_record_f32(const float *values, int64_t timestamp_us) {
  /* Reject invalid input and producer calls that fall outside sample instants.
   */
  if (values == NULL) {
    return false;
  }
  size_t sample_index = 0;
  if (!prepare_record(timestamp_us, &sample_index)) {
    return false;
  }

  /* Convert each physical value while preserving one invalid sentinel code. */
  size_t base = sample_index * recorder_config.channel_count;
  for (size_t channel = 0; channel < recorder_config.channel_count; channel++) {
    const esp_timeseries_channel_t *descriptor =
        &recorder_config.channels[channel];
    float encoded = (values[channel] - descriptor->offset) / descriptor->scale;
    if (!isfinite(encoded)) {
      recorder_buffer[base + channel] = ESP_TIMESERIES_INVALID_I16;
      invalid_counts[channel]++;
    } else if (encoded > (float)INT16_MAX) {
      recorder_buffer[base + channel] = INT16_MAX;
      saturation_counts[channel]++;
    } else if (encoded < (float)(INT16_MIN + 1)) {
      recorder_buffer[base + channel] = INT16_MIN + 1;
      saturation_counts[channel]++;
    } else {
      recorder_buffer[base + channel] = (int16_t)lroundf(encoded);
    }
  }

  /* Publish only after every channel in the record is complete. */
  finish_record(sample_index);
  return true;
}

/**
 * @brief Copy encoded values and publish a decimated record.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
bool esp_timeseries_record_i16(const int16_t *values, int64_t timestamp_us) {
  /* Share the same state transition and integer decimator as the float API. */
  if (values == NULL) {
    return false;
  }
  size_t sample_index = 0;
  if (!prepare_record(timestamp_us, &sample_index)) {
    return false;
  }

  /* Copy the complete record, then account for application invalid markers. */
  memcpy(&recorder_buffer[sample_index * recorder_config.channel_count], values,
         record_bytes());
  for (size_t channel = 0; channel < recorder_config.channel_count; channel++) {
    if (values[channel] == ESP_TIMESERIES_INVALID_I16) {
      invalid_counts[channel]++;
    }
  }

  /* Publish only after data and counters are updated. */
  finish_record(sample_index);
  return true;
}

/**
 * @brief Return a lock-free view of the current public state.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
esp_timeseries_state_t esp_timeseries_get_state(void) {
  /* Hide the private ARM configuration sentinel from API consumers. */
  int state = atomic_load_explicit(&recorder_state, memory_order_acquire);
  return state == RECORDER_STATE_CONFIGURING
             ? ESP_TIMESERIES_STATE_UNINITIALIZED
             : (esp_timeseries_state_t)state;
}

/**
 * @brief Assemble a diagnostic snapshot from static and atomic metadata.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
esp_err_t esp_timeseries_get_status(esp_timeseries_status_t *status) {
  /* Validate the output and reject unpublished configuration. */
  if (status == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_timeseries_state_t state = (esp_timeseries_state_t)atomic_load_explicit(
      &recorder_state, memory_order_acquire);
  if (state == ESP_TIMESERIES_STATE_UNINITIALIZED ||
      (int)state == RECORDER_STATE_CONFIGURING) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Copy one self-contained snapshot; active progress is intentionally live. */
  *status = (esp_timeseries_status_t){
      .state = state,
      .producer_rate_hz = recorder_config.producer_rate_hz,
      .sample_rate_hz =
          state == ESP_TIMESERIES_STATE_EMPTY ? 0U : recorder_sample_rate_hz,
      .sample_divider =
          state == ESP_TIMESERIES_STATE_EMPTY ? 0U : recorder_sample_divider,
      .capture_id = recorder_capture_id,
      .channel_count = recorder_config.channel_count,
      .buffer_bytes = sizeof(recorder_buffer),
      .record_bytes = record_bytes(),
      .sample_capacity = sample_capacity(),
      .sample_count =
          atomic_load_explicit(&recorder_sample_count, memory_order_acquire),
      .start_time_us = state == ESP_TIMESERIES_STATE_CAPTURING ||
                               state == ESP_TIMESERIES_STATE_FULL
                           ? recorder_start_time_us
                           : 0,
      .buffer_begin = (uintptr_t)&recorder_buffer[0],
      .buffer_end = (uintptr_t)&recorder_buffer[0] + sizeof(recorder_buffer),
  };
  return ESP_OK;
}

/**
 * @brief Publish a zero-copy view after the producer has frozen the payload.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
esp_err_t esp_timeseries_get_capture(esp_timeseries_capture_t *capture) {
  /* Only FULL guarantees immutable samples, counters, and capture metadata. */
  if (capture == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (atomic_load_explicit(&recorder_state, memory_order_acquire) !=
      ESP_TIMESERIES_STATE_FULL) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Acquire the published count before exposing zero-copy pointers. */
  size_t count =
      atomic_load_explicit(&recorder_sample_count, memory_order_acquire);
  *capture = (esp_timeseries_capture_t){
      .samples = recorder_buffer,
      .sample_count = count,
      .sample_capacity = sample_capacity(),
      .channel_count = recorder_config.channel_count,
      .payload_bytes = count * record_bytes(),
      .sample_rate_hz = recorder_sample_rate_hz,
      .capture_id = recorder_capture_id,
      .start_time_us = recorder_start_time_us,
      .channels = recorder_config.channels,
      .saturation_counts = saturation_counts,
      .invalid_counts = invalid_counts,
  };
  return ESP_OK;
}

/**
 * @brief Atomically release a completed capture without erasing its bytes.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
esp_err_t esp_timeseries_clear(void) {
  /* Claim FULL and invalidate all outstanding capture views in one transition.
   */
  int expected = ESP_TIMESERIES_STATE_FULL;
  if (!atomic_compare_exchange_strong_explicit(
          &recorder_state, &expected, ESP_TIMESERIES_STATE_EMPTY,
          memory_order_acq_rel, memory_order_acquire)) {
    return ESP_ERR_INVALID_STATE;
  }

  /* Reset logical progress; a subsequent ARM may overwrite the old bytes. */
  atomic_store_explicit(&recorder_sample_count, 0U, memory_order_release);
  return ESP_OK;
}

/**
 * @brief Convert a state enumeration to its protocol-facing name.
 * @see Declaration in esp_timeseries_recorder.h for the public API contract.
 */
const char *esp_timeseries_state_name(esp_timeseries_state_t state) {
  switch (state) {
  case ESP_TIMESERIES_STATE_UNINITIALIZED:
    return "UNINITIALIZED";
  case ESP_TIMESERIES_STATE_EMPTY:
    return "EMPTY";
  case ESP_TIMESERIES_STATE_ARMED:
    return "ARMED";
  case ESP_TIMESERIES_STATE_CAPTURING:
    return "CAPTURING";
  case ESP_TIMESERIES_STATE_FULL:
    return "FULL";
  default:
    return "UNKNOWN";
  }
}
