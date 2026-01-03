#!/usr/bin/env python3
"""
IMU Data Analysis for Frisbee Dog Catch Detection

This script:
1. Loads all CSV files from the data directory
2. Parses IMU time series and event labels
3. Aligns segments so t=0 is event time
4. Plots overlaid traces for different event types
5. Computes features per segment
6. Runs basic classification experiments
"""

import os
import glob
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from pathlib import Path
from collections import defaultdict
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
from scipy import stats

# Configuration
DATA_DIR = Path(__file__).parent.parent / "data" / "2026-01-03"
ANALYSIS_DIR = Path(__file__).parent  # Directory where this script is located
PLOT_WINDOW_SEC = 10  # Plot last N seconds before event
FEATURE_WINDOW_SEC = 3  # Compute features in last N seconds before event


def load_csv_file(filepath):
    """
    Load a single CSV file and extract IMU data and event label.
    
    Returns:
        tuple: (df_imu, event_label, event_time_ms, note)
            - df_imu: DataFrame with columns [t_ms, yaw, pitch, roll, ax, ay, az]
            - event_label: str ('catch', 'miss', 'other', or None)
            - event_time_ms: float (timestamp of the event in milliseconds)
            - note: str (optional note from the file)
    """
    try:
        df = pd.read_csv(filepath)
        
        # Find the event row (last non-empty event column)
        event_rows = df[df['event'].notna() & (df['event'] != '')]
        
        if len(event_rows) == 0:
            print(f"Warning: No event label found in {filepath}")
            return None, None, None, None
        
        # Get the last event row
        event_row = event_rows.iloc[-1]
        event_label = event_row['event'].strip().lower()
        note = event_row['note'] if pd.notna(event_row['note']) else ''
        
        # Get event timestamp (may be in t_ms column even if IMU data is empty)
        event_time_ms = pd.to_numeric(event_row['t_ms'], errors='coerce')
        if pd.isna(event_time_ms):
            # Fallback: use last IMU data timestamp
            df_imu_temp = df[df['event'].isna() | (df['event'] == '')].copy()
            if len(df_imu_temp) > 0:
                df_imu_temp['t_ms'] = pd.to_numeric(df_imu_temp['t_ms'], errors='coerce')
                event_time_ms = df_imu_temp['t_ms'].dropna().iloc[-1] if len(df_imu_temp['t_ms'].dropna()) > 0 else None
            else:
                print(f"Warning: No timestamp found for event in {filepath}")
                return None, None, None, None
        
        # Extract IMU data (all rows except the event row)
        df_imu = df[df['event'].isna() | (df['event'] == '')].copy()
        
        # Convert to numeric, handling any non-numeric values
        numeric_cols = ['t_ms', 'yaw', 'pitch', 'roll', 'ax', 'ay', 'az']
        for col in numeric_cols:
            df_imu[col] = pd.to_numeric(df_imu[col], errors='coerce')
        
        # Drop rows with NaN in critical columns
        df_imu = df_imu.dropna(subset=['t_ms', 'ax', 'ay', 'az'])
        
        if len(df_imu) == 0:
            print(f"Warning: No valid IMU data in {filepath}")
            return None, None, None, None
        
        return df_imu, event_label, event_time_ms, note
    
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
        return None, None, None


def align_to_event_time(df_imu, event_time_ms):
    """
    Align IMU data so that t=0 corresponds to the event time.
    
    Args:
        df_imu: DataFrame with t_ms column
        event_time_ms: Event timestamp in milliseconds
    
    Returns:
        DataFrame with t_rel_ms column (time relative to event, in ms)
    """
    df_aligned = df_imu.copy()
    df_aligned['t_rel_ms'] = df_aligned['t_ms'] - event_time_ms
    df_aligned['t_rel_sec'] = df_aligned['t_rel_ms'] / 1000.0
    return df_aligned


def compute_acceleration_magnitude(df):
    """Compute acceleration magnitude from ax, ay, az."""
    df = df.copy()
    df['accel_mag'] = np.sqrt(df['ax']**2 + df['ay']**2 + df['az']**2)
    return df


def extract_features(df_aligned, window_sec=FEATURE_WINDOW_SEC):
    """
    Extract features from the last N seconds before the event.
    
    Args:
        df_aligned: DataFrame with t_rel_sec column (negative values before event)
        window_sec: Time window in seconds before event to analyze
    
    Returns:
        dict: Dictionary of feature values
    """
    # Filter to window before event (t_rel_sec from -window_sec to 0)
    window_data = df_aligned[
        (df_aligned['t_rel_sec'] >= -window_sec) & 
        (df_aligned['t_rel_sec'] <= 0)
    ].copy()
    
    if len(window_data) == 0:
        return None
    
    # Ensure we have acceleration magnitude
    if 'accel_mag' not in window_data.columns:
        window_data = compute_acceleration_magnitude(window_data)
    
    features = {}
    
    # Vertical acceleration (az) features
    az = window_data['az'].values
    features['az_max'] = np.max(az)
    features['az_min'] = np.min(az)
    features['az_mean'] = np.mean(az)
    features['az_std'] = np.std(az)
    features['az_range'] = np.max(az) - np.min(az)
    features['az_peak_to_peak'] = np.ptp(az)
    
    # Acceleration magnitude features
    accel_mag = window_data['accel_mag'].values
    features['accel_mag_max'] = np.max(accel_mag)
    features['accel_mag_mean'] = np.mean(accel_mag)
    features['accel_mag_std'] = np.std(accel_mag)
    features['accel_mag_range'] = np.max(accel_mag) - np.min(accel_mag)
    
    # Count spikes (acceleration above threshold)
    threshold = np.mean(accel_mag) + 2 * np.std(accel_mag)
    features['spike_count'] = np.sum(accel_mag > threshold)
    
    # Orientation features (if available)
    if 'pitch' in window_data.columns:
        pitch = window_data['pitch'].values
        features['pitch_range'] = np.max(pitch) - np.min(pitch)
        features['pitch_std'] = np.std(pitch)
    
    if 'roll' in window_data.columns:
        roll = window_data['roll'].values
        features['roll_range'] = np.max(roll) - np.min(roll)
        features['roll_std'] = np.std(roll)
    
    # Time-based features
    if len(window_data) > 1:
        dt = np.diff(window_data['t_rel_ms'].values)
        features['mean_dt_ms'] = np.mean(dt)
        features['std_dt_ms'] = np.std(dt)
        features['sampling_rate_hz'] = 1000.0 / np.mean(dt) if np.mean(dt) > 0 else 0
    
    return features


def load_all_segments(data_dir=DATA_DIR):
    """
    Load all CSV files and return organized data.
    
    Returns:
        dict: {
            'catch': [list of DataFrames],
            'miss': [list of DataFrames],
            'other': [list of DataFrames]
        }
    """
    segments = defaultdict(list)
    files = sorted(glob.glob(str(data_dir / "*.txt")))
    
    print(f"Loading {len(files)} CSV files...")
    
    for filepath in files:
        df_imu, event_label, event_time_ms, note = load_csv_file(filepath)
        
        if df_imu is None or event_label is None or event_time_ms is None:
            continue
        
        # Align to event time
        df_aligned = align_to_event_time(df_imu, event_time_ms)
        df_aligned = compute_acceleration_magnitude(df_aligned)
        
        # Store with metadata
        df_aligned.attrs['event_label'] = event_label
        df_aligned.attrs['note'] = note
        df_aligned.attrs['filename'] = os.path.basename(filepath)
        df_aligned.attrs['event_time_ms'] = event_time_ms
        
        segments[event_label].append(df_aligned)
    
    print(f"Loaded segments: {dict((k, len(v)) for k, v in segments.items())}")
    return segments


def plot_overlaid_traces(segments, save_path=None):
    """
    Plot overlaid time series for different event types using Plotly.
    
    Args:
        segments: dict from load_all_segments()
        save_path: optional path to save HTML figure
    """
    # Create subplots
    fig = make_subplots(
        rows=3, cols=1,
        subplot_titles=('Vertical Acceleration (az) vs Time', 
                       'Acceleration Magnitude vs Time',
                       'Pitch vs Time'),
        vertical_spacing=0.08,
        shared_xaxes=True
    )
    
    colors = {'catch': 'green', 'miss': 'red', 'other': 'gray'}
    
    # Track which event types we've added for legend
    legend_added = {'catch': False, 'miss': False, 'other': False}
    
    for event_type, dfs in segments.items():
        color = colors.get(event_type, 'black')
        opacity = 0.3 if event_type == 'other' else 0.5
        
        for df in dfs:
            # Filter to plot window
            plot_data = df[
                (df['t_rel_sec'] >= -PLOT_WINDOW_SEC) & 
                (df['t_rel_sec'] <= 0)
            ]
            
            if len(plot_data) == 0:
                continue
            
            # Show legend only for first trace of each type
            showlegend = not legend_added.get(event_type, False)
            if showlegend:
                legend_added[event_type] = True
            
            # Vertical acceleration (az)
            fig.add_trace(
                go.Scatter(
                    x=plot_data['t_rel_sec'],
                    y=plot_data['az'],
                    mode='lines',
                    name=event_type,
                    line=dict(color=color, width=1),
                    opacity=opacity,
                    showlegend=showlegend,
                    legendgroup=event_type
                ),
                row=1, col=1
            )
            
            # Acceleration magnitude
            fig.add_trace(
                go.Scatter(
                    x=plot_data['t_rel_sec'],
                    y=plot_data['accel_mag'],
                    mode='lines',
                    name=event_type,
                    line=dict(color=color, width=1),
                    opacity=opacity,
                    showlegend=False,
                    legendgroup=event_type
                ),
                row=2, col=1
            )
            
            # Pitch
            fig.add_trace(
                go.Scatter(
                    x=plot_data['t_rel_sec'],
                    y=plot_data['pitch'],
                    mode='lines',
                    name=event_type,
                    line=dict(color=color, width=1),
                    opacity=opacity,
                    showlegend=False,
                    legendgroup=event_type
                ),
                row=3, col=1
            )
    
    # Add vertical line at event time (t=0)
    for row in [1, 2, 3]:
        fig.add_vline(
            x=0, 
            line_dash="dash", 
            line_color="black",
            line_width=1,
            annotation_text="Event",
            annotation_position="top",
            row=row, col=1
        )
    
    # Update axes labels
    fig.update_xaxes(title_text="Time Relative to Event (seconds)", row=3, col=1)
    fig.update_yaxes(title_text="az (m/s²)", row=1, col=1)
    fig.update_yaxes(title_text="Accel Magnitude (m/s²)", row=2, col=1)
    fig.update_yaxes(title_text="Pitch (degrees)", row=3, col=1)
    
    # Update layout
    fig.update_layout(
        height=900,
        title_text="IMU Data Traces by Event Type",
        title_x=0.5,
        title_font_size=16,
        hovermode='x unified',
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        )
    )
    
    if save_path:
        # Convert to Path object and ensure directory exists
        save_path = Path(save_path)
        if not save_path.is_absolute():
            # If relative path, make it relative to analysis directory
            save_path = ANALYSIS_DIR / save_path
        save_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Ensure .html extension
        if not save_path.suffix == '.html':
            save_path = save_path.with_suffix('.html')
        
        fig.write_html(str(save_path))
        print(f"Saved interactive plot to {save_path}")
    
    fig.show()


def compute_all_features(segments):
    """
    Compute features for all segments.
    
    Returns:
        tuple: (features_df, labels)
            - features_df: DataFrame with one row per segment
            - labels: array of event labels
    """
    all_features = []
    all_labels = []
    
    for event_type, dfs in segments.items():
        for df in dfs:
            features = extract_features(df, window_sec=FEATURE_WINDOW_SEC)
            if features is not None:
                all_features.append(features)
                all_labels.append(event_type)
    
    features_df = pd.DataFrame(all_features)
    labels = np.array(all_labels)
    
    return features_df, labels


def analyze_data_quality(segments):
    """Analyze sampling rate and data quality."""
    print("\n" + "="*60)
    print("DATA QUALITY ANALYSIS")
    print("="*60)
    
    all_dt = []
    all_lengths = []
    
    for event_type, dfs in segments.items():
        for df in dfs:
            if len(df) > 1:
                dt = np.diff(df['t_ms'].values)
                all_dt.extend(dt)
                all_lengths.append(len(df))
    
    if len(all_dt) > 0:
        print(f"\nSampling Rate Statistics:")
        print(f"  Mean Δt: {np.mean(all_dt):.2f} ms")
        print(f"  Std Δt: {np.std(all_dt):.2f} ms")
        print(f"  Mean sampling rate: {1000.0/np.mean(all_dt):.2f} Hz")
        print(f"  Min Δt: {np.min(all_dt):.2f} ms")
        print(f"  Max Δt: {np.max(all_dt):.2f} ms")
        
        # Check for gaps
        large_gaps = np.sum(np.array(all_dt) > 100)  # > 100ms
        print(f"\nData Gaps (>100ms): {large_gaps} occurrences")
        
        print(f"\nSegment Length Statistics:")
        print(f"  Mean length: {np.mean(all_lengths):.1f} samples")
        print(f"  Min length: {np.min(all_lengths)} samples")
        print(f"  Max length: {np.max(all_lengths)} samples")


def run_classification(features_df, labels):
    """
    Run basic classification experiments.
    
    Args:
        features_df: DataFrame of features
        labels: array of event labels
    """
    print("\n" + "="*60)
    print("CLASSIFICATION EXPERIMENTS")
    print("="*60)
    
    # Exclude non-numeric columns (like 'label' if it exists)
    numeric_cols = features_df.select_dtypes(include=[np.number]).columns
    features_numeric = features_df[numeric_cols]
    
    # Remove any NaN values
    valid_mask = ~features_numeric.isna().any(axis=1)
    features_clean = features_numeric[valid_mask].values
    labels_clean = labels[valid_mask]
    
    if len(features_clean) == 0:
        print("No valid features for classification!")
        return
    
    print(f"\nUsing {len(features_clean)} segments with {features_clean.shape[1]} features")
    print(f"Class distribution: {dict(zip(*np.unique(labels_clean, return_counts=True)))}")
    
    # Split data
    X_train, X_test, y_train, y_test = train_test_split(
        features_clean, labels_clean, test_size=0.3, random_state=42, stratify=labels_clean
    )
    
    # Train Random Forest
    print("\nTraining Random Forest Classifier...")
    rf = RandomForestClassifier(n_estimators=100, random_state=42, max_depth=10)
    rf.fit(X_train, y_train)
    
    # Evaluate
    train_score = rf.score(X_train, y_train)
    test_score = rf.score(X_test, y_test)
    
    print(f"\nTraining Accuracy: {train_score:.3f}")
    print(f"Test Accuracy: {test_score:.3f}")
    
    # Feature importance
    feature_names = features_df.columns
    importances = rf.feature_importances_
    indices = np.argsort(importances)[::-1]
    
    print("\nTop 10 Most Important Features:")
    for i in range(min(10, len(indices))):
        idx = indices[i]
        print(f"  {i+1}. {feature_names[idx]}: {importances[idx]:.4f}")
    
    # Classification report
    y_pred = rf.predict(X_test)
    print("\nClassification Report:")
    print(classification_report(y_test, y_pred))
    
    # Confusion matrix
    cm = confusion_matrix(y_test, y_pred)
    labels_unique = np.unique(labels_clean)
    
    fig = go.Figure(data=go.Heatmap(
        z=cm,
        x=labels_unique,
        y=labels_unique,
        colorscale='Blues',
        text=cm,
        texttemplate='%{text}',
        textfont={"size": 14},
        colorbar=dict(title="Count")
    ))
    
    fig.update_layout(
        title='Confusion Matrix',
        title_x=0.5,
        title_font_size=16,
        xaxis_title='Predicted Label',
        yaxis_title='True Label',
        width=600,
        height=600
    )
    
    save_path = ANALYSIS_DIR / 'confusion_matrix.html'
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(save_path))
    print(f"\nSaved confusion matrix to {save_path}")
    fig.show()


def summarize_features_by_class(features_df, labels):
    """Print summary statistics of features grouped by event type."""
    print("\n" + "="*60)
    print("FEATURE SUMMARY BY EVENT TYPE")
    print("="*60)
    
    # Work with a copy to avoid modifying the original
    features_copy = features_df.copy()
    features_copy['label'] = labels
    summary = features_copy.groupby('label').agg(['mean', 'std'])
    
    print("\nMean ± Std for key features:")
    key_features = ['az_max', 'az_range', 'accel_mag_max', 'spike_count']
    for feat in key_features:
        if feat in summary.columns:
            print(f"\n{feat}:")
            for label in summary.index:
                mean_val = summary.loc[label, (feat, 'mean')]
                std_val = summary.loc[label, (feat, 'std')]
                print(f"  {label:8s}: {mean_val:8.2f} ± {std_val:6.2f}")


def analyze_catch_vs_miss(features_df, labels):
    """
    Detailed analysis comparing catch vs miss events.
    
    Args:
        features_df: DataFrame of features
        labels: array of event labels
    """
    print("\n" + "="*60)
    print("CATCH vs MISS ANALYSIS")
    print("="*60)
    
    # Filter to only catch and miss
    catch_miss_mask = np.isin(labels, ['catch', 'miss'])
    features_cm = features_df[catch_miss_mask].copy()
    labels_cm = labels[catch_miss_mask]
    
    if len(features_cm) == 0:
        print("No catch or miss events found!")
        return None, None
    
    # Exclude non-numeric columns (like 'label' if it exists)
    numeric_cols = features_cm.select_dtypes(include=[np.number]).columns
    features_cm = features_cm[numeric_cols]
    
    print(f"\nDataset: {np.sum(labels_cm == 'catch')} catch, {np.sum(labels_cm == 'miss')} miss")
    
    # Statistical comparison for each feature
    print("\n" + "-"*60)
    print("STATISTICAL COMPARISON (Catch vs Miss)")
    print("-"*60)
    
    catch_data = features_cm[labels_cm == 'catch']
    miss_data = features_cm[labels_cm == 'miss']
    
    significant_features = []
    feature_stats = []
    
    for feature in features_cm.columns:
        catch_vals = catch_data[feature].dropna()
        miss_vals = miss_data[feature].dropna()
        
        if len(catch_vals) < 2 or len(miss_vals) < 2:
            continue
        
        # T-test
        try:
            t_stat, p_value = stats.ttest_ind(catch_vals, miss_vals)
        except:
            continue
        
        # Mann-Whitney U test (non-parametric)
        try:
            u_stat, u_pvalue = stats.mannwhitneyu(catch_vals, miss_vals, alternative='two-sided')
        except:
            u_pvalue = np.nan
        
        catch_mean = np.mean(catch_vals)
        miss_mean = np.mean(miss_vals)
        effect_size = (catch_mean - miss_mean) / np.sqrt(
            (np.var(catch_vals) + np.var(miss_vals)) / 2
        ) if (np.var(catch_vals) + np.var(miss_vals)) > 0 else 0
        
        feature_stats.append({
            'feature': feature,
            'catch_mean': catch_mean,
            'miss_mean': miss_mean,
            'difference': catch_mean - miss_mean,
            'p_value': p_value,
            'u_pvalue': u_pvalue,
            'effect_size': effect_size,
            'significant': p_value < 0.05
        })
        
        if p_value < 0.05:
            significant_features.append(feature)
    
    # Sort by p-value
    feature_stats_df = pd.DataFrame(feature_stats)
    feature_stats_df = feature_stats_df.sort_values('p_value')
    
    print("\nTop 10 Most Differentiating Features (by p-value):")
    print(f"{'Feature':<25} {'Catch Mean':<12} {'Miss Mean':<12} {'Diff':<10} {'p-value':<10} {'Effect Size':<12}")
    print("-" * 90)
    for _, row in feature_stats_df.head(10).iterrows():
        sig_marker = "***" if row['p_value'] < 0.001 else "**" if row['p_value'] < 0.01 else "*" if row['p_value'] < 0.05 else ""
        print(f"{row['feature']:<25} {row['catch_mean']:>11.2f} {row['miss_mean']:>11.2f} "
              f"{row['difference']:>9.2f} {row['p_value']:>9.4f}{sig_marker:>3} {row['effect_size']:>11.2f}")
    
    print("\nSignificance: *** p<0.001, ** p<0.01, * p<0.05")
    
    # Create comparison plots - use top features even if not statistically significant
    # (with small sample sizes, statistical significance is harder to achieve)
    features_to_plot = significant_features[:8] if len(significant_features) > 0 else feature_stats_df.head(8)['feature'].tolist()
    if len(features_to_plot) > 0:
        plot_catch_vs_miss_features(features_cm, labels_cm, features_to_plot)
    else:
        print("\nNo features available for plotting.")
    
    return features_cm, labels_cm


def plot_catch_vs_miss_features(features_df, labels, top_features):
    """
    Create box plots comparing catch vs miss for top differentiating features.
    
    Args:
        features_df: DataFrame of features (catch and miss only)
        labels: array of labels (catch/miss)
        top_features: list of feature names to plot
    """
    if len(top_features) == 0:
        print("\nNo significant features found for plotting.")
        return
    
    # Create subplots
    n_features = min(len(top_features), 8)
    n_cols = 2
    n_rows = (n_features + 1) // 2
    
    fig = make_subplots(
        rows=n_rows, cols=n_cols,
        subplot_titles=top_features[:n_features],
        vertical_spacing=0.12,
        horizontal_spacing=0.15
    )
    
    colors = {'catch': 'green', 'miss': 'red'}
    
    for idx, feature in enumerate(top_features[:n_features]):
        row = (idx // n_cols) + 1
        col = (idx % n_cols) + 1
        
        catch_vals = features_df[labels == 'catch'][feature].dropna()
        miss_vals = features_df[labels == 'miss'][feature].dropna()
        
        # Box plots
        fig.add_trace(
            go.Box(
                y=catch_vals,
                name='Catch',
                marker_color='green',
                boxmean='sd',
                showlegend=(idx == 0)
            ),
            row=row, col=col
        )
        
        fig.add_trace(
            go.Box(
                y=miss_vals,
                name='Miss',
                marker_color='red',
                boxmean='sd',
                showlegend=(idx == 0)
            ),
            row=row, col=col
        )
    
    fig.update_layout(
        height=300 * n_rows,
        title_text="Catch vs Miss: Feature Comparison",
        title_x=0.5,
        title_font_size=16,
        showlegend=True
    )
    
    save_path = ANALYSIS_DIR / 'catch_vs_miss_features.html'
    fig.write_html(str(save_path))
    print(f"\nSaved catch vs miss comparison plot to {save_path}")
    fig.show()


def run_catch_vs_miss_classification(features_df, labels):
    """
    Run classification specifically for catch vs miss (binary classification).
    
    Args:
        features_df: DataFrame of features
        labels: array of event labels
    """
    print("\n" + "="*60)
    print("CATCH vs MISS BINARY CLASSIFICATION")
    print("="*60)
    
    # Filter to only catch and miss
    catch_miss_mask = np.isin(labels, ['catch', 'miss'])
    features_cm = features_df[catch_miss_mask].copy()
    labels_cm = labels[catch_miss_mask]
    
    if len(features_cm) == 0:
        print("No catch or miss events found!")
        return
    
    # Exclude non-numeric columns (like 'label' if it exists)
    numeric_cols = features_cm.select_dtypes(include=[np.number]).columns
    features_cm = features_cm[numeric_cols]
    
    # Remove NaN values
    valid_mask = ~features_cm.isna().any(axis=1)
    features_clean = features_cm[valid_mask].values
    labels_clean = labels_cm[valid_mask]
    
    if len(features_clean) < 4:
        print("Not enough data for classification!")
        return
    
    print(f"\nDataset: {np.sum(labels_clean == 'catch')} catch, {np.sum(labels_clean == 'miss')} miss")
    
    # Split data
    X_train, X_test, y_train, y_test = train_test_split(
        features_clean, labels_clean, test_size=0.3, random_state=42, stratify=labels_clean
    )
    
    # Train Random Forest
    print("\nTraining Random Forest Classifier (Catch vs Miss)...")
    rf = RandomForestClassifier(n_estimators=100, random_state=42, max_depth=10)
    rf.fit(X_train, y_train)
    
    # Evaluate
    train_score = rf.score(X_train, y_train)
    test_score = rf.score(X_test, y_test)
    
    print(f"\nTraining Accuracy: {train_score:.3f}")
    print(f"Test Accuracy: {test_score:.3f}")
    
    # Feature importance
    feature_names = features_cm.columns  # Already filtered to numeric only
    importances = rf.feature_importances_
    indices = np.argsort(importances)[::-1]
    
    print("\nTop 10 Most Important Features for Catch vs Miss:")
    for i in range(min(10, len(indices))):
        idx = indices[i]
        print(f"  {i+1}. {feature_names[idx]}: {importances[idx]:.4f}")
    
    # Classification report
    y_pred = rf.predict(X_test)
    print("\nClassification Report (Catch vs Miss):")
    print(classification_report(y_test, y_pred))
    
    # Confusion matrix
    cm = confusion_matrix(y_test, y_pred)
    labels_unique = np.unique(labels_clean)
    
    fig = go.Figure(data=go.Heatmap(
        z=cm,
        x=labels_unique,
        y=labels_unique,
        colorscale='RdYlGn',
        text=cm,
        texttemplate='%{text}',
        textfont={"size": 16},
        colorbar=dict(title="Count")
    ))
    
    fig.update_layout(
        title='Confusion Matrix: Catch vs Miss',
        title_x=0.5,
        title_font_size=16,
        xaxis_title='Predicted Label',
        yaxis_title='True Label',
        width=500,
        height=500
    )
    
    save_path = ANALYSIS_DIR / 'catch_vs_miss_confusion_matrix.html'
    fig.write_html(str(save_path))
    print(f"\nSaved confusion matrix to {save_path}")
    fig.show()
    
    return rf, feature_names[indices[:10]]


def main():
    """Main analysis pipeline."""
    print("="*60)
    print("FRISBEE DOG IMU DATA ANALYSIS")
    print("="*60)
    
    # Load all segments
    segments = load_all_segments()
    
    if len(segments) == 0:
        print("No data loaded! Check data directory.")
        return
    
    # Data quality analysis
    analyze_data_quality(segments)
    
    # Plot overlaid traces
    print("\nGenerating plots...")
    plot_overlaid_traces(segments, save_path=ANALYSIS_DIR / 'overlaid_traces.html')
    
    # Extract features
    print("\nExtracting features...")
    features_df, labels = compute_all_features(segments)
    
    if len(features_df) == 0:
        print("No features extracted!")
        return
    
    print(f"\nExtracted {len(features_df)} feature vectors with {len(features_df.columns)} features each")
    
    # Summarize features
    summarize_features_by_class(features_df, labels)
    
    # Catch vs Miss specific analysis
    features_cm, labels_cm = analyze_catch_vs_miss(features_df, labels)
    
    if features_cm is not None:
        run_catch_vs_miss_classification(features_df, labels)
    
    # Run full classification (all classes)
    run_classification(features_df, labels)
    
    # Save features to CSV for further analysis
    features_df['label'] = labels
    features_csv_path = ANALYSIS_DIR / 'features.csv'
    features_df.to_csv(features_csv_path, index=False)
    print(f"\nSaved features to {features_csv_path}")


if __name__ == "__main__":
    main()

