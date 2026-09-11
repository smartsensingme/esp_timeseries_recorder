# ESP Time-Series Recorder

`esp_timeseries_recorder` stores uniformly sampled time series as interleaved
signed 16-bit records in a statically reserved, contiguous internal-DRAM
buffer. It is intended for deterministic ESP-IDF applications: the recording
path allocates no memory, writes no log, starts no task, and performs no I/O.

The component is deliberately limited to acquisition and storage. USB,
filesystems, CRCs, plotting, sensors, and control algorithms belong to other
components or to the application.

## Singleton, ownership, and concurrency

The recorder is a **singleton**: the component owns one global configuration,
one buffer, and one state machine, so its functions do not receive an instance
handle. It supports sequential captures, not simultaneous independent captures.

Use one producer context for both `esp_timeseries_record_f32()` and
`esp_timeseries_record_i16()`. Consumer tasks may query state/status at any
time. They may access samples only through `esp_timeseries_get_capture()` while
the recorder remains `FULL`. Calling `esp_timeseries_clear()` invalidates every
previous capture view.

`esp_timeseries_init()` shallow-copies the configuration. The channel array and
its strings must therefore remain valid for the program lifetime; static
storage is the normal choice.

## Configuration

The compile-time options are:

- `CONFIG_ESP_TIMESERIES_RECORDER_BUFFER_KIB`: internal DRAM reserved for the
  sample buffer;
- `CONFIG_ESP_TIMESERIES_RECORDER_MAX_CHANNELS`: upper bound used for the
  per-channel invalid/saturation counters.

At runtime, `esp_timeseries_config_t` defines:

- `producer_rate_hz`: how often the application calls a record function;
- `channel_count`: values in each record;
- `channels`: descriptors in the exact order used by every submitted sample.

Each `esp_timeseries_channel_t` supplies `name`, `unit`, `scale`, `offset`, and
an optional `encoding`. For the floating-point API, conversion is:

```text
raw = round((real_value - offset) / scale)
real_value = raw * scale + offset
```

For example, `scale = 0.1` and `offset = 0` represent 612.3 rpm as raw 6123.
Quantization error is at most approximately half a scale count when no clamp
occurs. `scale` must be positive and all scale/offset values must be finite.

`INT16_MIN` is reserved as `ESP_TIMESERIES_INVALID_I16`. Non-finite float input
uses that code. Finite overflow is clamped to `INT16_MIN + 1` or `INT16_MAX` and
increments the channel saturation counter. The raw API copies values unchanged
and counts occurrences of the reserved invalid code.

`encoding` does not change recorder behavior. It lets an application and host
agree on meanings for special raw codes, such as current-sense status flags.

## Rates and capacity

Each capture selects a sample rate at runtime. It must divide
`producer_rate_hz` exactly:

```text
sample_divider = producer_rate_hz / sample_rate_hz
```

Thus a 1 kHz producer accepts 1000, 500, 250, 200, 125, or 100 Hz, but not
300 Hz. Integer decimation makes sample spacing uniform: the first producer
call is stored, then every `sample_divider` calls.

Capacity and duration are:

```text
record_bytes    = channel_count * sizeof(int16_t)
sample_capacity = floor(buffer_bytes / record_bytes)
duration_s      = sample_capacity / sample_rate_hz
```

With 128 KiB, five channels, and 500 Hz, capacity is 13,107 records and the
capture lasts about 26.214 seconds. Unused trailing bytes are harmless.

## State machine

| State | Meaning | Allowed next operation |
|---|---|---|
| `UNINITIALIZED` | Initialization not complete | `esp_timeseries_init()` |
| `EMPTY` | No readable capture | `esp_timeseries_arm()` |
| `ARMED` | Waiting for first producer call | A record function starts capture |
| `CAPTURING` | Decimating and writing records | Continue producer calls |
| `FULL` | Immutable capture available | Read/DUMP, then `esp_timeseries_clear()` |

The buffer becoming full stops acquisition automatically. It never wraps and
does not overwrite the capture until the consumer explicitly clears and arms
again. Buffer contents are volatile and disappear on reset or power loss.

## Complete example

```c
static const esp_timeseries_channel_t channels[] = {
    {.name = "speed", .unit = "rpm", .scale = 0.1f, .offset = 0.0f},
    {.name = "control", .unit = "%", .scale = 0.01f, .offset = 0.0f},
};

void recorder_setup(void)
{
    const esp_timeseries_config_t config = {
        .producer_rate_hz = 1000,
        .channel_count = 2,
        .channels = channels,
    };
    ESP_ERROR_CHECK(esp_timeseries_init(&config));
    ESP_ERROR_CHECK(esp_timeseries_arm(500));
}

/* Called by exactly one producer at 1 kHz. */
void control_iteration(float speed_rpm, float control_percent, int64_t now_us)
{
    const float values[] = {speed_rpm, control_percent};
    (void)esp_timeseries_record_f32(values, now_us);
}

void consume_when_full(void)
{
    esp_timeseries_capture_t capture;
    if (esp_timeseries_get_capture(&capture) == ESP_OK) {
        /* Read or transmit capture.samples before CLEAR. */
        ESP_ERROR_CHECK(esp_timeseries_clear());
    }
}
```

The return value of a record function says whether that particular producer
call stored a record; `false` is normal while empty, armed-rate decimation is
skipping a call, or the capture is full.

## API summary

| Function | Role |
|---|---|
| `esp_timeseries_init()` | Validate and publish the singleton configuration |
| `esp_timeseries_arm()` | Select the next capture rate and reset its metadata |
| `esp_timeseries_record_f32()` | Quantize physical values and conditionally store |
| `esp_timeseries_record_i16()` | Conditionally store already encoded values |
| `esp_timeseries_get_state()` | Fast lock-free state query |
| `esp_timeseries_get_status()` | Copy progress and buffer geometry |
| `esp_timeseries_get_capture()` | Obtain a zero-copy immutable FULL view |
| `esp_timeseries_clear()` | Release FULL and return to EMPTY |
| `esp_timeseries_state_name()` | Convert a state to a protocol/diagnostic string |

Pair this component with `esp_timeseries_usb_transport` when a host must arm,
inspect, and download captures over native USB.
