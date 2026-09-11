/**
 * @file esp_timeseries_recorder.h
 * @brief Deterministic singleton recorder for interleaved 16-bit time series.
 *
 * The component owns one statically allocated internal-DRAM buffer. A single
 * producer may submit records from a periodic loop while consumer tasks inspect
 * state and read a completed, immutable capture. It performs no allocation,
 * logging, transport, or blocking operation in the recording path.
 *
 * Configuration and channel descriptors are shallow-copied. Their referenced
 * storage, including all strings, must remain valid for the recorder lifetime.
 */
#ifndef ESP_TIMESERIES_RECORDER_H_
#define ESP_TIMESERIES_RECORDER_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Reserved raw value used to represent an invalid or non-finite sample. */
#define ESP_TIMESERIES_INVALID_I16 INT16_MIN

/** Recorder lifecycle states exposed to producer and consumer code. */
typedef enum {
  ESP_TIMESERIES_STATE_UNINITIALIZED = 0, /**< init has not completed. */
  ESP_TIMESERIES_STATE_EMPTY, /**< No readable capture; ARM is allowed. */
  ESP_TIMESERIES_STATE_ARMED, /**< Next producer call establishes time zero. */
  ESP_TIMESERIES_STATE_CAPTURING, /**< Records are being decimated and stored.
                                   */
  ESP_TIMESERIES_STATE_FULL, /**< Capture is full, immutable, and readable. */
} esp_timeseries_state_t;

/** Metadata defining the conversion of one signed 16-bit channel. */
typedef struct {
  const char
      *name; /**< Human-readable name; CR/LF is unsafe for text protocols. */
  const char *unit; /**< Physical unit, such as "rpm", "A", or "%". */
  float scale;      /**< Physical units per raw count; finite and positive. */
  float offset;     /**< Physical value represented by raw zero; finite. */
  const char
      *encoding; /**< Optional application-defined raw-code interpretation. */
} esp_timeseries_channel_t;

/** Initialization parameters shared by every capture of the singleton. */
typedef struct {
  uint32_t producer_rate_hz; /**< Record-function call frequency, in hertz. */
  size_t channel_count;      /**< Number of int16 values in each record. */
  const esp_timeseries_channel_t *channels; /**< Persistent descriptor array. */
} esp_timeseries_config_t;

/** Snapshot of recorder metadata and progress for diagnostics or protocols. */
typedef struct {
  esp_timeseries_state_t state; /**< State observed when the snapshot began. */
  uint32_t producer_rate_hz;    /**< Configured producer-call rate. */
  uint32_t sample_rate_hz;      /**< Capture rate, or zero while EMPTY. */
  uint32_t sample_divider;      /**< Producer calls per stored record. */
  uint32_t capture_id;    /**< Identifier incremented by each successful ARM. */
  size_t channel_count;   /**< Values stored in each record. */
  size_t buffer_bytes;    /**< Compile-time buffer reservation. */
  size_t record_bytes;    /**< Bytes occupied by one complete record. */
  size_t sample_capacity; /**< Maximum complete records in the buffer. */
  size_t sample_count;    /**< Complete records published by the producer. */
  int64_t start_time_us;  /**< Timestamp of the first producer call. */
  uintptr_t buffer_begin; /**< Inclusive address, for diagnostics only. */
  uintptr_t buffer_end;   /**< Exclusive address, for diagnostics only. */
} esp_timeseries_status_t;

/** Read-only view of a capture, valid only while state remains FULL. */
typedef struct {
  const int16_t
      *samples;        /**< Sample-interleaved payload in native byte order. */
  size_t sample_count; /**< Number of complete records in samples. */
  size_t sample_capacity; /**< Maximum records for the current configuration. */
  size_t channel_count;   /**< Values per record. */
  size_t payload_bytes;   /**< Number of readable bytes at samples. */
  uint32_t sample_rate_hz; /**< Uniform stored-record rate. */
  uint32_t capture_id;     /**< Identifier assigned when capture was armed. */
  int64_t
      start_time_us; /**< Timestamp supplied with the first producer call. */
  const esp_timeseries_channel_t *channels; /**< Persistent descriptors. */
  const uint32_t *saturation_counts; /**< Per-channel finite clamp counts. */
  const uint32_t *invalid_counts;    /**< Per-channel reserved-code counts. */
} esp_timeseries_capture_t;

/**
 * @brief Initialize the singleton and publish it in the EMPTY state.
 *
 * Called externally during application startup; no component function calls it.
 * It validates descriptors, shallow-copies @p config, clears counters, and
 * verifies that at least one complete record fits in the static buffer.
 *
 * @param config Persistent producer and channel configuration.
 * @return ESP_OK; ESP_ERR_INVALID_ARG; ESP_ERR_INVALID_STATE if already
 *         initialized; or ESP_ERR_NO_MEM if no complete record fits.
 */
esp_err_t esp_timeseries_init(const esp_timeseries_config_t *config);

/**
 * @brief Prepare an EMPTY recorder for a new capture.
 *
 * Called externally by an application or transport. The rate must be nonzero,
 * at most producer_rate_hz, and an exact integer divisor of that rate. The next
 * recording call begins capture and supplies its start timestamp.
 *
 * @param sample_rate_hz Desired stored-record rate in hertz.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, or ESP_ERR_INVALID_STATE.
 */
esp_err_t esp_timeseries_arm(uint32_t sample_rate_hz);

/**
 * @brief Quantize and conditionally store one producer-rate sample.
 *
 * Called externally from the single producer loop. Internally calls the private
 * prepare_record() and finish_record() helpers. Encoding is
 * round((value-offset)/scale); non-finite values use the reserved code and
 * finite overflow is clamped and counted. The path does not block or allocate.
 *
 * @param values channel_count physical values in descriptor order.
 * @param timestamp_us Producer timestamp; used only by the first call after
 * ARM.
 * @return true only when this call stored and published a complete record.
 */
bool esp_timeseries_record_f32(const float *values, int64_t timestamp_us);

/**
 * @brief Conditionally store one already-encoded producer-rate sample.
 *
 * Called externally from the single producer loop and uses the same private
 * decimation/publication helpers as esp_timeseries_record_f32(). Values are
 * copied unchanged; reserved invalid values are counted.
 *
 * @param values channel_count raw values in descriptor order.
 * @param timestamp_us Producer timestamp; used only by the first call after
 * ARM.
 * @return true only when this call stored and published a complete record.
 */
bool esp_timeseries_record_i16(const int16_t *values, int64_t timestamp_us);

/**
 * @brief Read the public state without locking.
 *
 * Called only externally, including from producer hot paths. A private
 * transient configuration state is conservatively reported as UNINITIALIZED.
 */
esp_timeseries_state_t esp_timeseries_get_state(void);

/**
 * @brief Copy recorder configuration, progress, and buffer diagnostics.
 *
 * Called externally by status consumers; the USB transport uses it for STATUS
 * and DUMP. Fields may be approximate while CAPTURING but are stable when FULL.
 *
 * @param[out] status Destination snapshot.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, or ESP_ERR_INVALID_STATE before init.
 */
esp_err_t esp_timeseries_get_status(esp_timeseries_status_t *status);

/**
 * @brief Obtain a zero-copy view of the immutable completed capture.
 *
 * Called externally by consumers such as the USB transport. Returned pointers
 * remain valid only until esp_timeseries_clear() succeeds.
 *
 * @param[out] capture Destination view.
 * @return ESP_OK, ESP_ERR_INVALID_ARG, or ESP_ERR_INVALID_STATE unless FULL.
 */
esp_err_t esp_timeseries_get_capture(esp_timeseries_capture_t *capture);

/**
 * @brief Release a FULL capture and return the recorder to EMPTY.
 *
 * Called only externally. It invalidates capture views and resets the logical
 * count without erasing buffer bytes.
 *
 * @return ESP_OK or ESP_ERR_INVALID_STATE unless FULL.
 */
esp_err_t esp_timeseries_clear(void);

/**
 * @brief Map a public state to a stable uppercase protocol string.
 *
 * Called externally; the USB transport uses it in status responses. The
 * returned string literal must not be modified or freed.
 */
const char *esp_timeseries_state_name(esp_timeseries_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* ESP_TIMESERIES_RECORDER_H_ */
