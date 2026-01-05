#!/usr/bin/env python3
"""
Rename CSV files to include their event classification and notes.

Example:
  download.txt -> download_other.txt
  download(20).txt -> download(20)_catch.txt
  download(41).txt -> download(41)_other_walking_home_sniffing.txt
"""

import os
import pandas as pd
from pathlib import Path
import re

DATA_DIR = Path(__file__).parent.parent / "data" / "2026-01-04"


def extract_event_info(filepath):
    """
    Extract event label and note from a CSV file.
    
    Returns:
        tuple: (event_label, note) or (None, None) if not found
    """
    try:
        df = pd.read_csv(filepath)
        
        # Find the event row (last non-empty event column)
        event_rows = df[df['event'].notna() & (df['event'] != '')]
        
        if len(event_rows) == 0:
            return None, None
        
        # Get the last event row
        event_row = event_rows.iloc[-1]
        event_label = event_row['event'].strip().lower() if pd.notna(event_row['event']) else None
        note = event_row['note'].strip() if pd.notna(event_row['note']) else ''
        
        return event_label, note
    
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None, None


def sanitize_filename_part(text):
    """
    Sanitize text to be safe for use in filenames.
    Replaces spaces and special chars with underscores.
    """
    if not text:
        return ""
    # Replace spaces and special characters with underscores
    text = re.sub(r'[^\w\-]', '_', text)
    # Remove multiple consecutive underscores
    text = re.sub(r'_+', '_', text)
    # Remove leading/trailing underscores
    text = text.strip('_')
    return text.lower()


def rename_file(filepath, event_label, note):
    """
    Rename a file to include event label and note.
    
    Args:
        filepath: Path to the file
        event_label: Event classification (catch, miss, other)
        note: Optional note text
    """
    if event_label is None:
        print(f"  Skipping {filepath.name} - no event label found")
        return False
    
    # Get original filename parts
    stem = filepath.stem  # filename without extension
    suffix = filepath.suffix  # .txt
    
    # Build new filename
    parts = [stem, event_label]
    
    # Add note if present
    if note:
        note_clean = sanitize_filename_part(note)
        if note_clean:
            parts.append(note_clean)
    
    new_name = '_'.join(parts) + suffix
    new_path = filepath.parent / new_name
    
    # Check if target already exists
    if new_path.exists() and new_path != filepath:
        print(f"  Warning: Target {new_name} already exists, skipping {filepath.name}")
        return False
    
    # Rename
    try:
        filepath.rename(new_path)
        print(f"  {filepath.name} -> {new_name}")
        return True
    except Exception as e:
        print(f"  Error renaming {filepath.name}: {e}")
        return False


def main():
    """Main function to rename all files."""
    print("="*60)
    print("RENAMING FILES BY EVENT CLASSIFICATION")
    print("="*60)
    print(f"\nData directory: {DATA_DIR}")
    
    if not DATA_DIR.exists():
        print(f"Error: Data directory not found: {DATA_DIR}")
        return
    
    # Get all .txt files
    files = sorted(DATA_DIR.glob("*.txt"))
    
    if len(files) == 0:
        print("No .txt files found!")
        return
    
    print(f"\nFound {len(files)} files to process\n")
    
    renamed_count = 0
    skipped_count = 0
    
    for filepath in files:
        # Skip files that already have event labels in the name
        if '_catch' in filepath.stem or '_miss' in filepath.stem or '_other' in filepath.stem:
            print(f"  Skipping {filepath.name} - already renamed")
            skipped_count += 1
            continue
        
        # Extract event info
        event_label, note = extract_event_info(filepath)
        
        if event_label is None:
            print(f"  Skipping {filepath.name} - no event label found")
            skipped_count += 1
            continue
        
        # Rename
        if rename_file(filepath, event_label, note):
            renamed_count += 1
    
    print(f"\n" + "="*60)
    print(f"Summary: {renamed_count} renamed, {skipped_count} skipped")
    print("="*60)


if __name__ == "__main__":
    main()


