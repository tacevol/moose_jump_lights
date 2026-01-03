
# Frisbee Dog IMU Data – Analysis Context

I’m working on a dog-mounted IMU project to detect frisbee-related events (especially **catch** vs **miss**) from motion data.

## Hardware / Firmware Setup

- **MCU:** Seeed XIAO ESP32C3  
- **IMU:** Adafruit BNO085 in UART-RVC mode  
- **Sampling:** Target ~100 Hz; there is some jitter and occasional short gaps  
- **Logging pattern:**
  - The ESP32 keeps a rolling **RAM ring buffer** of IMU data (~30 seconds).
  - I manually press an event button on a web UI when I see my dog **catch**, **miss**, or an **other** interesting event.
  - When I press the button, the device writes the **last ~30 seconds** from the ring buffer to a CSV file as a “segment”, then resumes rolling logging in RAM.

## Data Collected

- About **40 CSV files** captured at the park in one session.
- Each file represents a **single 30-second “event window”** ending at the moment I pressed one of:
  - `catch`
  - `miss`
  - `other`
- Typical behavior around a “catch” window:
  - Dog runs around me, sometimes circles or sits.
  - I give a release/OK cue and throw the frisbee.
  - Dog accelerates, jumps, and (hopefully) catches the frisbee.
- So each segment should include some “prelude” motion (run-up, turns, sitting) and the jump/landing itself.

## CSV Format

Each CSV has a header like:

```text
t_ms,yaw,pitch,roll,ax,ay,az,event,note
```

Where:

- `t_ms` = device timestamp in milliseconds  
- `yaw,pitch,roll` = orientation angles from the BNO085 (degrees)  
- `ax,ay,az` = acceleration in m/s² (including gravity)  
- `event` = usually empty; last row includes the event label  
- `note` = optional free-text note (usually empty for now)

At the end of each file, there should be a final row with `event` set to `catch`, `miss`, or `other` (plus any `note` text if I entered one). That marks the **flag time** when I pressed the button; all previous rows are the pre-event history.

## Goals With This Data

### 1. Verify Data Quality and Timing

- Confirm actual sampling rate (average Hz) and distribution of inter-sample intervals (`Δt_ms`).
- Check for large gaps or corrupt segments.
- Get a sense of noise level in `ax, ay, az` and the orientation channels.

### 2. Understand Motion Patterns Around Events

- For each file, align the data so that `t = 0` corresponds to the **event row** (button press).
- Plot time-series around the event (e.g., last 5–10 seconds before the event) for:
  - `az` (vertical acceleration)
  - Acceleration magnitude `sqrt(ax^2 + ay^2 + az^2)`
  - Possibly pitch/roll
- Visually compare **catch**, **miss**, and **other** segments to see if catches have a recognizable signature (e.g., jump/landing spike pattern).

### 3. Feature Extraction and Sanity-Checking

- For each segment, compute candidate features in a fixed window before the event (e.g., last 2–3 seconds), such as:
  - Max / min / std of `az`
  - Peak-to-peak range in `az`
  - Number of large acceleration spikes above some threshold
  - Basic features on acceleration magnitude
- Summarize these features per event type (`catch`, `miss`, `other`) to see if they look separable at all.

### 4. Early Classification Experiments (Offline)

- Using the segment-level features plus labels (`catch`/`miss`/`other`), try simple models:
  - Threshold rules
  - Maybe a small tree / random forest
- The goal is **not** to build a huge model, but to learn whether:
  - There *is* a consistent “catch” signature in the IMU data, and  
  - The signal-to-noise ratio is high enough to justify a real-time detector later.

### 5. Future Constraint (For Later, But Important)

- Any eventual real-time detector will need to run on the XIAO ESP32C3 (TinyML / lightweight model / simple thresholds).
- During analysis we should prefer features and models that are implementable on-device (no heavy models, no massive window buffers).

## What I Want Cursor to Help With

- Load **all ~40 CSV files** from a directory.
- Parse out:
  - The IMU time series (`t_ms, yaw, pitch, roll, ax, ay, az`)
  - The **event label** from the final row (`catch`, `miss`, `other`).
- Build helpers to:
  - Align each segment so `t = 0` is event time.
  - Plot a bunch of overlaid traces for `az` and/or acceleration magnitude for each label class.
  - Compute and tabulate simple features per segment.
  - Run basic classification experiments to see if “catch” vs “miss” vs “other” are distinguishable at all from those features.
