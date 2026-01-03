# IMU Data Analysis

This directory contains scripts for analyzing the frisbee dog IMU data to detect catch vs miss events.

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

Run the main analysis script:
```bash
python analyze_imu_data.py
```

## What the Script Does

1. **Loads all CSV files** from `../data/2026-01-03/`
2. **Parses IMU data and event labels** (catch, miss, other)
3. **Aligns segments** so t=0 corresponds to the event time
4. **Analyzes data quality** (sampling rate, gaps, etc.)
5. **Plots interactive overlaid traces** for different event types (using Plotly):
   - Vertical acceleration (az)
   - Acceleration magnitude
   - Pitch angle
   - Interactive features: zoom, pan, hover tooltips, legend toggling
6. **Extracts features** from the last 3 seconds before each event:
   - Max/min/mean/std of vertical acceleration
   - Acceleration magnitude statistics
   - Spike counts
   - Orientation features
7. **Runs classification experiments** using Random Forest to distinguish catch vs miss vs other
8. **Saves outputs**:
   - `overlaid_traces.html` - Interactive time series plots (open in browser)
   - `confusion_matrix.html` - Interactive confusion matrix (open in browser)
   - `features.csv` - All extracted features for further analysis

## Configuration

You can modify these constants at the top of `analyze_imu_data.py`:
- `PLOT_WINDOW_SEC`: Time window for plots (default: 10 seconds)
- `FEATURE_WINDOW_SEC`: Time window for feature extraction (default: 3 seconds)
- `DATA_DIR`: Path to data directory

