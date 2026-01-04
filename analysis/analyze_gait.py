#!/usr/bin/env python3
"""
Gait Analysis - Detect step cadence in walking IMU data.

The dog walks at approximately 3 steps per second (3 Hz cadence).
This script analyzes walking data to detect this periodic pattern.
"""

import os
import glob
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pathlib import Path
from scipy import signal
from scipy.fft import fft, fftfreq

DATA_DIR = Path(__file__).parent.parent / "data" / "2026-01-03"
ANALYSIS_DIR = Path(__file__).parent

# Expected cadence: 3 steps per second = 3 Hz
EXPECTED_CADENCE_HZ = 3.0
CADENCE_TOLERANCE_HZ = 0.5  # ±0.5 Hz tolerance


def load_walking_file(filepath):
    """
    Load a walking CSV file and extract IMU data.
    
    Returns:
        DataFrame with IMU data, or None if error
    """
    try:
        df = pd.read_csv(filepath)
        
        # Extract IMU data (exclude event rows)
        df_imu = df[df['event'].isna() | (df['event'] == '')].copy()
        
        # Convert to numeric
        numeric_cols = ['t_ms', 'yaw', 'pitch', 'roll', 'ax', 'ay', 'az']
        for col in numeric_cols:
            df_imu[col] = pd.to_numeric(df_imu[col], errors='coerce')
        
        # Drop rows with NaN in critical columns
        df_imu = df_imu.dropna(subset=['t_ms', 'ax', 'ay', 'az'])
        
        if len(df_imu) == 0:
            return None
        
        # Convert time to seconds
        df_imu['t_sec'] = df_imu['t_ms'] / 1000.0
        
        # Compute acceleration magnitude
        df_imu['accel_mag'] = np.sqrt(df_imu['ax']**2 + df_imu['ay']**2 + df_imu['az']**2)
        
        return df_imu
    
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
        return None


def detect_step_cadence(df, signal_col='accel_mag', min_freq=1.0, max_freq=6.0):
    """
    Detect step cadence using FFT analysis.
    
    Args:
        df: DataFrame with time series data
        signal_col: Column name to analyze
        min_freq: Minimum frequency to search (Hz)
        max_freq: Maximum frequency to search (Hz)
    
    Returns:
        dict with cadence analysis results
    """
    if len(df) < 100:
        return None
    
    # Get time and signal
    t = df['t_sec'].values
    y = df[signal_col].values
    
    # Remove DC component (mean)
    y = y - np.mean(y)
    
    # Apply high-pass filter to remove very low frequency drift
    dt = np.mean(np.diff(t))
    fs = 1.0 / dt  # Sampling frequency
    
    # If sampling is irregular, resample
    if np.std(np.diff(t)) > 0.1 * dt:
        t_uniform = np.arange(t[0], t[-1], dt)
        y = np.interp(t_uniform, t, y)
        t = t_uniform
    
    # Apply window to reduce spectral leakage
    window = signal.windows.hann(len(y))
    y_windowed = y * window
    
    # Compute FFT
    n = len(y_windowed)
    yf = fft(y_windowed)
    xf = fftfreq(n, dt)
    
    # Get positive frequencies only
    positive_freq_mask = xf > 0
    freqs = xf[positive_freq_mask]
    power = np.abs(yf[positive_freq_mask])**2
    
    # Find peak in expected cadence range
    cadence_mask = (freqs >= min_freq) & (freqs <= max_freq)
    if not np.any(cadence_mask):
        return None
    
    cadence_freqs = freqs[cadence_mask]
    cadence_power = power[cadence_mask]
    
    # Find dominant frequency
    peak_idx = np.argmax(cadence_power)
    dominant_freq = cadence_freqs[peak_idx]
    peak_power = cadence_power[peak_idx]
    
    # Also check for fundamental near expected cadence (might be half of detected)
    expected_range_mask = (freqs >= EXPECTED_CADENCE_HZ - CADENCE_TOLERANCE_HZ) & \
                         (freqs <= EXPECTED_CADENCE_HZ + CADENCE_TOLERANCE_HZ)
    if np.any(expected_range_mask):
        expected_power = power[expected_range_mask]
        expected_peak_idx = np.argmax(expected_power)
        expected_freq = freqs[expected_range_mask][expected_peak_idx]
        expected_peak_power = expected_power[expected_peak_idx]
    else:
        expected_freq = None
        expected_peak_power = 0
    
    # Check if peak is significant (above noise floor)
    noise_floor = np.percentile(power, 50)  # Median as noise estimate
    signal_to_noise = peak_power / noise_floor if noise_floor > 0 else 0
    
    # Determine if we should use expected frequency or dominant
    # If expected frequency has significant power, prefer it
    if expected_freq and expected_peak_power > 0.3 * peak_power:
        best_freq = expected_freq
        best_power = expected_peak_power
    else:
        best_freq = dominant_freq
        best_power = peak_power
    
    return {
        'dominant_freq_hz': dominant_freq,
        'best_freq_hz': best_freq,
        'expected_freq_hz': expected_freq,
        'peak_power': best_power,
        'signal_to_noise': signal_to_noise,
        'freqs': freqs,
        'power': power,
        'sampling_rate_hz': fs,
        'duration_sec': t[-1] - t[0]
    }


def find_steps_autocorrelation(df, signal_col='accel_mag'):
    """
    Find step timing using autocorrelation.
    
    Args:
        df: DataFrame with time series data
        signal_col: Column name to analyze
    
    Returns:
        dict with step detection results
    """
    if len(df) < 200:
        return None
    
    # Get signal
    y = df[signal_col].values
    t = df['t_sec'].values
    
    # Remove DC component
    y = y - np.mean(y)
    
    # Compute autocorrelation
    autocorr = np.correlate(y, y, mode='full')
    autocorr = autocorr[len(autocorr)//2:]
    
    # Normalize
    autocorr = autocorr / autocorr[0]
    
    # Find peaks in autocorrelation (periodic pattern)
    # Look for peaks between 0.2s and 1.0s (2-5 Hz)
    dt = np.mean(np.diff(t))
    min_lag_samples = int(0.2 / dt)  # 0.2 seconds
    max_lag_samples = int(1.0 / dt)  # 1.0 seconds
    
    if max_lag_samples >= len(autocorr):
        max_lag_samples = len(autocorr) - 1
    
    search_range = autocorr[min_lag_samples:max_lag_samples]
    peaks, properties = signal.find_peaks(search_range, height=0.3, distance=int(0.15/dt))
    
    if len(peaks) == 0:
        return None
    
    # Get the first significant peak (fundamental period)
    first_peak_idx = peaks[0] + min_lag_samples
    period_samples = first_peak_idx
    period_sec = period_samples * dt
    cadence_hz = 1.0 / period_sec if period_sec > 0 else 0
    
    return {
        'cadence_hz': cadence_hz,
        'period_sec': period_sec,
        'autocorr': autocorr,
        'lags_sec': np.arange(len(autocorr)) * dt
    }


def analyze_walking_file(filepath):
    """
    Analyze a single walking file for gait patterns.
    
    Returns:
        dict with analysis results
    """
    df = load_walking_file(filepath)
    if df is None:
        return None
    
    results = {
        'filename': os.path.basename(filepath),
        'duration_sec': df['t_sec'].iloc[-1] - df['t_sec'].iloc[0],
        'num_samples': len(df),
        'sampling_rate_hz': 1000.0 / np.mean(np.diff(df['t_ms'].values))
    }
    
    # FFT analysis on acceleration magnitude
    fft_results = detect_step_cadence(df, signal_col='accel_mag')
    if fft_results:
        results.update({
            'fft_cadence_hz': fft_results['best_freq_hz'],
            'fft_dominant_hz': fft_results['dominant_freq_hz'],
            'fft_expected_hz': fft_results.get('expected_freq_hz'),
            'fft_snr': fft_results['signal_to_noise'],
            'fft_freqs': fft_results['freqs'],
            'fft_power': fft_results['power']
        })
    
    # Also analyze vertical acceleration (az) - often better for gait
    fft_az_results = detect_step_cadence(df, signal_col='az')
    if fft_az_results:
        results.update({
            'fft_az_cadence_hz': fft_az_results['best_freq_hz'],
            'fft_az_freqs': fft_az_results['freqs'],
            'fft_az_power': fft_az_results['power']
        })
    
    # Analyze orientation data (pitch, yaw, roll)
    for orientation_col in ['pitch', 'yaw', 'roll']:
        if orientation_col in df.columns:
            fft_orient_results = detect_step_cadence(df, signal_col=orientation_col)
            if fft_orient_results:
                results.update({
                    f'fft_{orientation_col}_cadence_hz': fft_orient_results['best_freq_hz'],
                    f'fft_{orientation_col}_freqs': fft_orient_results['freqs'],
                    f'fft_{orientation_col}_power': fft_orient_results['power']
                })
    
    # Autocorrelation analysis
    autocorr_results = find_steps_autocorrelation(df, signal_col='accel_mag')
    if autocorr_results:
        results.update({
            'autocorr_cadence_hz': autocorr_results['cadence_hz'],
            'autocorr_period_sec': autocorr_results['period_sec']
        })
    
    results['df'] = df  # Store dataframe for plotting
    
    return results


def plot_gait_analysis(results_list, save_path=None):
    """
    Create interactive plots for gait analysis - one plot per file for readability.
    
    Args:
        results_list: List of analysis result dicts
        save_path: Base path to save HTML files (will append filename)
    """
    n_files = len(results_list)
    if n_files == 0:
        print("No data to plot!")
        return
    
    saved_files = []
    
    for idx, results in enumerate(results_list):
        df = results['df']
        filename = results['filename']
        
        # Create subplots: 6 rows stacked vertically
        # (accel mag, az, pitch, yaw, roll, FFT)
        fig = make_subplots(
            rows=6, cols=1,
            subplot_titles=(
                f"{filename} - Acceleration Magnitude",
                f"{filename} - Vertical Acceleration (az)",
                f"{filename} - Pitch",
                f"{filename} - Yaw",
                f"{filename} - Roll",
                f"{filename} - FFT Spectrum (All Signals)"
            ),
            vertical_spacing=0.08
        )
        
        # Time series - acceleration magnitude
        fig.add_trace(
            go.Scatter(
                x=df['t_sec'],
                y=df['accel_mag'],
                mode='lines',
                name='Accel Magnitude',
                line=dict(width=1, color='blue'),
                showlegend=False
            ),
            row=1, col=1
        )
        
        # Time series - vertical acceleration (az)
        fig.add_trace(
            go.Scatter(
                x=df['t_sec'],
                y=df['az'],
                mode='lines',
                name='az',
                line=dict(width=1, color='green'),
                showlegend=False
            ),
            row=2, col=1
        )
        
        # Time series - pitch
        if 'pitch' in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df['t_sec'],
                    y=df['pitch'],
                    mode='lines',
                    name='Pitch',
                    line=dict(width=1, color='orange'),
                    showlegend=False
                ),
                row=3, col=1
            )
        
        # Time series - yaw
        if 'yaw' in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df['t_sec'],
                    y=df['yaw'],
                    mode='lines',
                    name='Yaw',
                    line=dict(width=1, color='purple'),
                    showlegend=False
                ),
                row=4, col=1
            )
        
        # Time series - roll
        if 'roll' in df.columns:
            fig.add_trace(
                go.Scatter(
                    x=df['t_sec'],
                    y=df['roll'],
                    mode='lines',
                    name='Roll',
                    line=dict(width=1, color='red'),
                    showlegend=False
                ),
                row=5, col=1
            )
        
        # FFT spectrum - overlay all signals
        row_fft = 6
        legend_added = False
        
        # Acceleration magnitude FFT
        if 'fft_freqs' in results:
            freqs = results['fft_freqs']
            power = results['fft_power']
            display_mask = freqs <= 10.0
            freqs_display = freqs[display_mask]
            power_display = power[display_mask]
            
            fig.add_trace(
                go.Scatter(
                    x=freqs_display,
                    y=power_display,
                    mode='lines',
                    name='Accel Mag',
                    line=dict(width=2, color='blue'),
                    showlegend=not legend_added
                ),
                row=row_fft, col=1
            )
            if not legend_added:
                legend_added = True
        
        # Vertical acceleration (az) FFT
        if 'fft_az_freqs' in results:
            freqs_az = results['fft_az_freqs']
            power_az = results['fft_az_power']
            display_mask_az = freqs_az <= 10.0
            freqs_az_display = freqs_az[display_mask_az]
            power_az_display = power_az[display_mask_az]
            
            fig.add_trace(
                go.Scatter(
                    x=freqs_az_display,
                    y=power_az_display,
                    mode='lines',
                    name='az',
                    line=dict(width=1.5, color='green', dash='dash'),
                    showlegend=not legend_added
                ),
                row=row_fft, col=1
            )
            if not legend_added:
                legend_added = True
        
        # Orientation FFTs
        for orient, color in [('pitch', 'orange'), ('yaw', 'purple'), ('roll', 'red')]:
            if f'fft_{orient}_freqs' in results:
                freqs_orient = results[f'fft_{orient}_freqs']
                power_orient = results[f'fft_{orient}_power']
                display_mask_orient = freqs_orient <= 10.0
                freqs_orient_display = freqs_orient[display_mask_orient]
                power_orient_display = power_orient[display_mask_orient]
                
                fig.add_trace(
                    go.Scatter(
                        x=freqs_orient_display,
                        y=power_orient_display,
                        mode='lines',
                        name=orient.capitalize(),
                        line=dict(width=1, color=color, dash='dot'),
                        showlegend=not legend_added
                    ),
                    row=row_fft, col=1
                )
                if not legend_added:
                    legend_added = True
        
        # Mark expected cadence
        fig.add_vline(
            x=EXPECTED_CADENCE_HZ,
            line_dash="dash",
            line_color="red",
            line_width=2,
            annotation_text="Expected 3 Hz",
            annotation_position="top",
            row=row_fft, col=1
        )
        
        # Mark detected cadence from accel mag
        if 'fft_cadence_hz' in results:
            fig.add_vline(
                x=results['fft_cadence_hz'],
                line_dash="dot",
                line_color="green",
                line_width=2,
                annotation_text=f"Detected {results['fft_cadence_hz']:.2f} Hz",
                annotation_position="top",
                row=row_fft, col=1
            )
        
        # Mark expected range peak if available
        if 'fft_expected_hz' in results and results['fft_expected_hz']:
            fig.add_vline(
                x=results['fft_expected_hz'],
                line_dash="dashdot",
                line_color="orange",
                line_width=1.5,
                annotation_text=f"Range Peak {results['fft_expected_hz']:.2f} Hz",
                annotation_position="bottom",
                row=row_fft, col=1
            )
        
        # Update axes labels
        fig.update_xaxes(title_text="Time (s)", row=1, col=1)
        fig.update_yaxes(title_text="Accel Magnitude (m/s²)", row=1, col=1)
        fig.update_xaxes(title_text="Time (s)", row=2, col=1)
        fig.update_yaxes(title_text="az (m/s²)", row=2, col=1)
        fig.update_xaxes(title_text="Time (s)", row=3, col=1)
        fig.update_yaxes(title_text="Pitch (degrees)", row=3, col=1)
        fig.update_xaxes(title_text="Time (s)", row=4, col=1)
        fig.update_yaxes(title_text="Yaw (degrees)", row=4, col=1)
        fig.update_xaxes(title_text="Time (s)", row=5, col=1)
        fig.update_yaxes(title_text="Roll (degrees)", row=5, col=1)
        fig.update_xaxes(title_text="Frequency (Hz)", row=6, col=1)
        fig.update_yaxes(title_text="Power (log scale)", row=6, col=1, type="log")  # Log scale for power
        
        fig.update_layout(
            height=1500,
            title_text=f"Gait Analysis: {filename}",
            title_x=0.5,
            title_font_size=14,
            showlegend=True,
            legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
        )
        
        # Save individual file
        if save_path:
            save_path_obj = Path(save_path)
            save_path_obj.parent.mkdir(parents=True, exist_ok=True)
            
            # Create filename-safe version
            safe_filename = filename.replace('(', '_').replace(')', '_').replace('.txt', '')
            individual_path = save_path_obj.parent / f"{save_path_obj.stem}_{safe_filename}.html"
            
            fig.write_html(str(individual_path))
            saved_files.append(individual_path)
        
        # Show plot (will open in browser)
        fig.show()
    
    # Also create a summary plot with all FFT spectra overlaid
    if len(results_list) > 1:
        fig_summary = go.Figure()
        
        for results in results_list:
            if 'fft_freqs' in results:
                freqs = results['fft_freqs']
                power = results['fft_power']
                display_mask = freqs <= 10.0
                
                fig_summary.add_trace(
                    go.Scatter(
                        x=freqs[display_mask],
                        y=power[display_mask],
                        mode='lines',
                        name=results['filename'],
                        line=dict(width=1.5)
                    )
                )
        
        fig_summary.add_vline(
            x=EXPECTED_CADENCE_HZ,
            line_dash="dash",
            line_color="red",
            line_width=2,
            annotation_text="Expected 3 Hz"
        )
        
        fig_summary.update_layout(
            title="Gait Analysis: FFT Spectra Comparison (All Files)",
            title_x=0.5,
            xaxis_title="Frequency (Hz)",
            yaxis_title="Power (log scale)",
            yaxis_type="log",
            height=600,
            width=1000
        )
        
        if save_path:
            save_path_obj = Path(save_path)
            summary_path = save_path_obj.parent / f"{save_path_obj.stem}_summary.html"
            fig_summary.write_html(str(summary_path))
            saved_files.append(summary_path)
        
        fig_summary.show()
    
    if save_path and saved_files:
        print(f"\nSaved {len(saved_files)} plot files:")
        for f in saved_files:
            print(f"  {f}")


def analyze_activity_files(filepaths, activity_name):
    """
    Analyze a set of files for a specific activity type.
    
    Args:
        filepaths: List of file paths to analyze
        activity_name: Name of the activity (e.g., "walking", "running_catch", "running_miss")
    
    Returns:
        tuple: (results_list, summary_data)
    """
    if len(filepaths) == 0:
        return [], []
    
    print(f"\nFound {len(filepaths)} {activity_name} files:\n")
    for f in filepaths:
        print(f"  {f.name}")
    
    print(f"\n" + "="*60)
    print(f"ANALYZING {activity_name.upper()} FILES...")
    print("="*60 + "\n")
    
    results_list = []
    summary_data = []
    
    for filepath in filepaths:
        print(f"Analyzing: {filepath.name}")
        results = analyze_walking_file(filepath)  # Same analysis function works for all
        
        if results is None:
            print(f"  Failed to analyze\n")
            continue
        
        results_list.append(results)
        
        # Print summary
        summary = {
            'filename': results['filename'],
            'activity': activity_name,
            'duration_sec': results['duration_sec'],
            'sampling_rate_hz': results['sampling_rate_hz']
        }
        
        if 'fft_cadence_hz' in results:
            summary['fft_cadence_hz'] = results['fft_cadence_hz']
            summary['fft_snr'] = results['fft_snr']
            print(f"  FFT Cadence (mag): {results['fft_cadence_hz']:.2f} Hz (SNR: {results['fft_snr']:.1f})")
            if 'fft_expected_hz' in results and results['fft_expected_hz']:
                print(f"    Expected range peak: {results['fft_expected_hz']:.2f} Hz")
        
        if 'fft_az_cadence_hz' in results:
            summary['fft_az_cadence_hz'] = results['fft_az_cadence_hz']
            print(f"  FFT Cadence (az): {results['fft_az_cadence_hz']:.2f} Hz")
        
        # Orientation cadences
        for orient in ['pitch', 'yaw', 'roll']:
            if f'fft_{orient}_cadence_hz' in results:
                summary[f'fft_{orient}_cadence_hz'] = results[f'fft_{orient}_cadence_hz']
                print(f"  FFT Cadence ({orient}): {results[f'fft_{orient}_cadence_hz']:.2f} Hz")
        
        if 'autocorr_cadence_hz' in results:
            summary['autocorr_cadence_hz'] = results['autocorr_cadence_hz']
            print(f"  Autocorr Cadence: {results['autocorr_cadence_hz']:.2f} Hz")
        
        summary_data.append(summary)
        print()
    
    return results_list, summary_data


def main():
    """Main analysis function."""
    print("="*60)
    print("GAIT ANALYSIS - Step Cadence Detection")
    print("="*60)
    print(f"\nExpected walking cadence: {EXPECTED_CADENCE_HZ} Hz (3 steps/second, 2-beat gait)")
    print(f"Data directory: {DATA_DIR}\n")
    
    # Find files by activity type
    walking_files = sorted(DATA_DIR.glob("*walking*.txt"))
    catch_files = sorted(DATA_DIR.glob("*_catch.txt"))
    miss_files = sorted(DATA_DIR.glob("*_miss.txt"))
    
    all_results = []
    all_summary = []
    
    # Analyze walking files (2-beat gait)
    if len(walking_files) > 0:
        walking_results, walking_summary = analyze_activity_files(walking_files, "walking")
        all_results.extend(walking_results)
        all_summary.extend(walking_summary)
    
    # Analyze catch files (running/jumping)
    if len(catch_files) > 0:
        catch_results, catch_summary = analyze_activity_files(catch_files, "running_catch")
        all_results.extend(catch_results)
        all_summary.extend(catch_summary)
    
    # Analyze miss files (running/jumping)
    if len(miss_files) > 0:
        miss_results, miss_summary = analyze_activity_files(miss_files, "running_miss")
        all_results.extend(miss_results)
        all_summary.extend(miss_summary)
    
    if len(all_summary) == 0:
        print("No files found to analyze!")
        return
    
    # Print summary table grouped by activity
    print("="*60)
    print("SUMMARY BY ACTIVITY")
    print("="*60)
    
    for activity in ["walking", "running_catch", "running_miss"]:
        activity_data = [s for s in all_summary if s.get('activity') == activity]
        if len(activity_data) == 0:
            continue
        
        print(f"\n{activity.upper().replace('_', ' ')} ({len(activity_data)} files):")
        print(f"{'Filename':<40} {'Duration':<10} {'FFT Mag':<10} {'FFT Az':<10} {'SNR':<8}")
        print("-" * 90)
        for s in activity_data:
            fft_cad = f"{s.get('fft_cadence_hz', 0):.2f}" if 'fft_cadence_hz' in s else "N/A"
            fft_az = f"{s.get('fft_az_cadence_hz', 0):.2f}" if 'fft_az_cadence_hz' in s else "N/A"
            snr = f"{s.get('fft_snr', 0):.1f}" if 'fft_snr' in s else "N/A"
            print(f"{s['filename']:<40} {s['duration_sec']:>8.1f}s {fft_cad:>8} {fft_az:>8} {snr:>6}")
        
        # Average cadence for this activity
        cadences = [s.get('fft_cadence_hz') for s in activity_data if 'fft_cadence_hz' in s]
        if len(cadences) > 0:
            avg_cadence = np.mean(cadences)
            print(f"\n  Average cadence: {avg_cadence:.2f} Hz")
            if activity == "walking":
                print(f"  Expected (2-beat): {EXPECTED_CADENCE_HZ} Hz")
                print(f"  Difference: {abs(avg_cadence - EXPECTED_CADENCE_HZ):.2f} Hz")
    
    # Create plots for each activity type (using already analyzed results)
    walking_results_plot = [r for r in all_results if any(f.name == r['filename'] for f in walking_files)]
    catch_results_plot = [r for r in all_results if any(f.name == r['filename'] for f in catch_files)]
    miss_results_plot = [r for r in all_results if any(f.name == r['filename'] for f in miss_files)]
    
    if len(walking_results_plot) > 0:
        print("\nGenerating walking plots...")
        plot_gait_analysis(walking_results_plot, save_path=ANALYSIS_DIR / 'gait_analysis_walking.html')
    
    if len(catch_results_plot) > 0:
        print("\nGenerating catch/running plots...")
        plot_gait_analysis(catch_results_plot, save_path=ANALYSIS_DIR / 'gait_analysis_catch.html')
    
    if len(miss_results_plot) > 0:
        print("\nGenerating miss/running plots...")
        plot_gait_analysis(miss_results_plot, save_path=ANALYSIS_DIR / 'gait_analysis_miss.html')


if __name__ == "__main__":
    main()

