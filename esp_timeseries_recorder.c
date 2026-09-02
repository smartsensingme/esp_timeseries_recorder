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

static size_t record_bytes(void) {
  return recorder_config.channel_count * sizeof(int16_t);
}

static size_t sample_capacity(void) {
  size_t bytes = record_bytes();
  return bytes > 0U ? sizeof(recorder_buffer) / bytes : 0U;
}

esp_err_t esp_timeseries_init(const esp_timeseries_config_t *config) {
  if (config == NULL || config->channels == NULL ||
      config->producer_rate_hz == 0U || config->channel_count == 0U ||
      config->channel_count > CONFIG_ESP_TIMESERIES_RECORDER_MAX_CHANNELS) {
    return ESP_ERR_INVALID_ARG;
  }
  if (atomic_load_explicit(&recorder_state, memory_order_acquire) !=
      ESP_TIMESERIES_STATE_UNINITIALIZED) {
    return ESP_ERR_INVALID_STATE;
  }
  for (size_t channel = 0; channel < config->channel_count; channel++) {
    if (config->channels[channel].name == NULL ||
        config->channels[channel].unit == NULL ||
        !isfinite(config->channels[channel].scale) ||
        config->channels[channel].scale <= 0.0f ||
        !isfinite(config->channels[channel].offset)) {
      return ESP_ERR_INVALID_ARG;
    }
  }

  recorder_config = *config;
  if (sample_capacity() == 0U) {
    recorder_config = (esp_timeseries_config_t){0};
    return ESP_ERR_NO_MEM;
  }
  memset(saturation_counts, 0, sizeof(saturation_counts));
  memset(invalid_counts, 0, sizeof(invalid_counts));
  atomic_store_explicit(&recorder_sample_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&recorder_state, ESP_TIMESERIES_STATE_EMPTY,
                        memory_order_release);
  return ESP_OK;
}

esp_err_t esp_timeseries_arm(uint32_t sample_rate_hz) {
  if (sample_rate_hz == 0U ||
      sample_rate_hz > recorder_config.producer_rate_hz ||
      recorder_config.producer_rate_hz % sample_rate_hz != 0U) {
    return ESP_ERR_INVALID_ARG;
  }
  int expected = ESP_TIMESERIES_STATE_EMPTY;
  if (!atomic_compare_exchange_strong_explicit(
          &recorder_state, &expected, RECORDER_STATE_CONFIGURING,
          memory_order_acq_rel, memory_order_acquire)) {
    return ESP_ERR_INVALID_STATE;
  }

  recorder_sample_rate_hz = sample_rate_hz;
  recorder_sample_divider = recorder_config.producer_rate_hz / sample_rate_hz;
  recorder_capture_id++;
  producer_calls_until_sample = 0U;
  recorder_start_time_us = 0;
  memset(saturation_counts, 0, sizeof(saturation_counts));
  memset(invalid_counts, 0, sizeof(invalid_counts));
  atomic_store_explicit(&recorder_sample_count, 0U, memory_order_relaxed);
  atomic_store_explicit(&recorder_state, ESP_TIMESERIES_STATE_ARMED,
                        memory_order_release);
  return ESP_OK;
}

static bool prepare_record(int64_t timestamp_us, size_t *sample_index) {
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
  if (producer_calls_until_sample > 0U) {
    producer_calls_until_sample--;
    return false;
  }
  producer_calls_until_sample = recorder_sample_divider - 1U;
  *sample_index =
      atomic_load_explicit(&recorder_sample_count, memory_order_relaxed);
  return *sample_index < sample_capacity();
}

static void finish_record(size_t sample_index) {
  size_t count = sample_index + 1U;
  atomic_store_explicit(&recorder_sample_count, count, memory_order_release);
  if (count >= sample_capacity()) {
    atomic_store_explicit(&recorder_state, ESP_TIMESERIES_STATE_FULL,
                          memory_order_release);
  }
}

bool esp_timeseries_record_f32(const float *values, int64_t timestamp_us) {
  if (values == NULL) {
    return false;
  }
  size_t sample_index = 0;
  if (!prepare_record(timestamp_us, &sample_index)) {
    return false;
  }

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
  finish_record(sample_index);
  return true;
}

bool esp_timeseries_record_i16(const int16_t *values, int64_t timestamp_us) {
  if (values == NULL) {
    return false;
  }
  size_t sample_index = 0;
  if (!prepare_record(timestamp_us, &sample_index)) {
    return false;
  }
  memcpy(&recorder_buffer[sample_index * recorder_config.channel_count], values,
         record_bytes());
  for (size_t channel = 0; channel < recorder_config.channel_count; channel++) {
    if (values[channel] == ESP_TIMESERIES_INVALID_I16) {
      invalid_counts[channel]++;
    }
  }
  finish_record(sample_index);
  return true;
}

esp_timeseries_state_t esp_timeseries_get_state(void) {
  int state = atomic_load_explicit(&recorder_state, memory_order_acquire);
  return state == RECORDER_STATE_CONFIGURING
             ? ESP_TIMESERIES_STATE_UNINITIALIZED
             : (esp_timeseries_state_t)state;
}

esp_err_t esp_timeseries_get_status(esp_timeseries_status_t *status) {
  if (status == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_timeseries_state_t state = (esp_timeseries_state_t)atomic_load_explicit(
      &recorder_state, memory_order_acquire);
  if (state == ESP_TIMESERIES_STATE_UNINITIALIZED ||
      (int)state == RECORDER_STATE_CONFIGURING) {
    return ESP_ERR_INVALID_STATE;
  }

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

esp_err_t esp_timeseries_get_capture(esp_timeseries_capture_t *capture) {
  if (capture == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (atomic_load_explicit(&recorder_state, memory_order_acquire) !=
      ESP_TIMESERIES_STATE_FULL) {
    return ESP_ERR_INVALID_STATE;
  }

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

esp_err_t esp_timeseries_clear(void) {
  int expected = ESP_TIMESERIES_STATE_FULL;
  if (!atomic_compare_exchange_strong_explicit(
          &recorder_state, &expected, ESP_TIMESERIES_STATE_EMPTY,
          memory_order_acq_rel, memory_order_acquire)) {
    return ESP_ERR_INVALID_STATE;
  }
  atomic_store_explicit(&recorder_sample_count, 0U, memory_order_release);
  return ESP_OK;
}

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
