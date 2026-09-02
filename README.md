# ESP Time-Series Recorder

Reusable singleton recorder for deterministic ESP-IDF applications. It stores
sample-major, interleaved `int16_t` records in one statically reserved,
contiguous internal-DRAM buffer.

The application declares channel names, units, scale, and offset. For the
floating-point API, reconstruction is:

```text
real_value = int16_value * scale + offset
```

`INT16_MIN` is reserved as an invalid/non-finite sample marker. The component
also exposes per-channel invalid and saturation counters for transport
metadata; finite negative saturation therefore uses `INT16_MIN + 1`.

The producer rate is fixed at initialization. Each capture selects a runtime
sample rate that must divide the producer rate exactly. For a 1 kHz producer,
valid examples include 1000, 500, 250, 200, 125, and 100 Hz. Exact division
keeps every interval uniform and avoids a fractional-rate scheduler.

The state machine is `EMPTY -> ARMED -> CAPTURING -> FULL -> EMPTY`. A full
capture remains immutable until `esp_timeseries_clear()`, so a transport can
retry after a CRC failure. The component deliberately contains no USB,
console, filesystem, CRC, motor-control, or task code.

`CONFIG_ESP_TIMESERIES_RECORDER_BUFFER_KIB` defines the memory reservation. The
sample capacity is calculated at initialization as:

```text
buffer_bytes / (channel_count * sizeof(int16_t))
```

The buffer is volatile. Reset or power loss destroys its contents.
