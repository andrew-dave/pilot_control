#!/usr/bin/env python3
"""
Data Folder Setup Script
Creates hierarchical folder structure for organized data collection:
- Day folder: Month_Day_Year (e.g., November_5_2025)
- Section folder: Section_N_HHMMSS (increments each launch)
- Data folders: Visual_data and GPR_scan_data
"""

import os
import sys
from datetime import datetime
from pathlib import Path
import json


def get_day_folder_name():
    """Generate day folder name: Month_Day_Year"""
    now = datetime.now()
    return now.strftime("%B_%d_%Y")


def find_next_section_number(day_folder_path):
    """Find the next available section number for today"""
    if not day_folder_path.exists():
        return 1
    
    # Find all Section_* folders
    section_folders = [d for d in day_folder_path.iterdir() 
                      if d.is_dir() and d.name.startswith("Section_")]
    
    if not section_folders:
        return 1
    
    # Extract section numbers
    section_numbers = []
    for folder in section_folders:
        try:
            # Extract number from "Section_N_HHMMSS"
            parts = folder.name.split("_")
            if len(parts) >= 2:
                section_num = int(parts[1])
                section_numbers.append(section_num)
        except (ValueError, IndexError):
            continue
    
    if not section_numbers:
        return 1
    
    return max(section_numbers) + 1


def setup_day_folder_only(base_directory):
    """
    Create only the day folder (Month_Day_Year). No section folder is created.
    Section folders are created dynamically by data_collection_coordinator when
    /dc/start is called (using current time at start, not launch time).
    
    Returns:
        dict: base_directory, day_folder, day_name
    """
    base_path = Path(base_directory).expanduser()
    day_folder_name = get_day_folder_name()
    day_folder = base_path / day_folder_name
    day_folder.mkdir(parents=True, exist_ok=True)
    return {
        'base_directory': str(base_path),
        'day_folder': str(day_folder),
        'day_name': day_folder_name,
    }


def setup_data_folders(base_directory):
    """
    Create the hierarchical folder structure and return paths.
    Used for standalone runs; launch uses setup_day_folder_only() and
    data_collection_coordinator creates sections on /dc/start.
    
    Returns:
        dict: Contains paths for visual_data and gpr_scan_data
    """
    base_path = Path(base_directory).expanduser()
    
    # Create day folder
    day_folder_name = get_day_folder_name()
    day_folder = base_path / day_folder_name
    day_folder.mkdir(parents=True, exist_ok=True)
    
    # Find next section number
    section_num = find_next_section_number(day_folder)
    
    # Create section folder with timestamp
    timestamp = datetime.now().strftime("%H%M%S")
    section_folder_name = f"Section_{section_num}_{timestamp}"
    section_folder = day_folder / section_folder_name
    section_folder.mkdir(parents=True, exist_ok=True)
    
    # Create data subfolders
    visual_data_folder = section_folder / "Visual_data"
    gpr_scan_folder = section_folder / "GPR_scan_data"
    
    visual_data_folder.mkdir(exist_ok=True)
    gpr_scan_folder.mkdir(exist_ok=True)
    
    paths = {
        'base_directory': str(base_path),
        'day_folder': str(day_folder),
        'section_folder': str(section_folder),
        'visual_data_folder': str(visual_data_folder),
        'gpr_scan_folder': str(gpr_scan_folder),
        'section_number': section_num,
        'timestamp': timestamp,
        'day_name': day_folder_name,
        'section_name': section_folder_name
    }
    
    return paths


def main():
    """Main function - can be called from launch file or standalone"""
    # Default base directory
    default_base = os.path.join(os.path.expanduser('~'), 'robot_data')
    
    # Allow override from command line
    base_directory = sys.argv[1] if len(sys.argv) > 1 else default_base
    
    # Setup folders
    paths = setup_data_folders(base_directory)
    
    # Print paths (for launch file to capture)
    print(f"DAY_FOLDER={paths['day_folder']}")
    print(f"SECTION_FOLDER={paths['section_folder']}")
    print(f"VISUAL_DATA_FOLDER={paths['visual_data_folder']}")
    print(f"GPR_SCAN_FOLDER={paths['gpr_scan_folder']}")
    
    # Also save to a config file for nodes to read
    config_file = Path(paths['section_folder']) / 'session_config.json'
    with open(config_file, 'w') as f:
        json.dump(paths, f, indent=2)
    
    print(f"\n✓ Created session structure:")
    print(f"  Day: {paths['day_name']}")
    print(f"  Section: {paths['section_name']}")
    print(f"  Visual data: {paths['visual_data_folder']}")
    print(f"  GPR data: {paths['gpr_scan_folder']}")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

