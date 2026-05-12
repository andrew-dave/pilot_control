#!/usr/bin/env python3
"""
finalize_mission_local.py — robot-side standalone mission finaliser.

Used by the OCU's "Finalize via SSH (offline)" path in the
LinkHealthMonitor disconnect-resilience flow.  When Zenoh is dead and
data_collection_coordinator's /dc/finalize_mission is unreachable, the
OCU SSHs into the robot and invokes this script directly.  No ROS, no
rclpy import — just rescans the latest open Mission_HHMMSS folder under
/R_DATA, reconciles its sections with what's actually on disk, and
writes mission_finalized_at + finalized_via=ssh_offline atomically into
mission_config.json.

The robot-side data_collection_coordinator's auto-finalize watchdog
will eventually do the same thing (10 min idle) — this script just
lets the operator force the same recovery path on demand.

Resolution:
  1. If --mission-folder is given, finalize that folder.
  2. Else search /R_DATA for the most recently modified Mission_*
     folder whose mission_config.json has mission_finalized_at == null.
  3. Else exit 0 with no-op JSON ({"finalized": false, "reason": ...}).

Usage:
  finalize_mission_local.py [--base-dir /R_DATA] [--mission-folder PATH]

Output (stdout, single JSON line):
  {"finalized": true, "mission_folder": "/R_DATA/...",
   "finalized_at": "2026-...", "section_count": N,
   "warnings": [...]}

Exit codes:
  0  success (finalized OR no-op)
  2  bad CLI args
  3  mission folder unwritable / structurally broken
"""

import argparse
import json
import os
import sys
from datetime import datetime
from pathlib import Path


def atomic_write_json(path: Path, data: dict) -> None:
    """tmp + os.replace() so a crash mid-write doesn't corrupt the file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    with open(tmp, 'w') as f:
        json.dump(data, f, indent=2)
        f.flush()
        try:
            os.fsync(f.fileno())
        except OSError:
            pass
    os.replace(tmp, path)


def find_open_mission(base_dir: Path) -> Path:
    """Find the most recently-modified Mission_* folder with no
    mission_finalized_at set.  Returns Path or None."""
    if not base_dir.exists():
        return None
    candidates = []
    # Layout: <base>/<day>/<building>/Mission_HHMMSS/mission_config.json
    for cfg in base_dir.glob('*/*/Mission_*/mission_config.json'):
        try:
            with open(cfg, 'r') as f:
                data = json.load(f)
            if data.get('mission_finalized_at') is None:
                candidates.append((cfg.stat().st_mtime, cfg.parent))
        except (OSError, json.JSONDecodeError):
            continue
    if not candidates:
        return None
    candidates.sort(reverse=True)
    return candidates[0][1]


def finalize_mission(mission_folder: Path) -> dict:
    """Finalize a single mission folder.  Reconciles sections with what's
    on disk, then atomically updates mission_config.json."""
    cfg_path = mission_folder / 'mission_config.json'
    if not cfg_path.exists():
        raise RuntimeError(f'mission_config.json missing in {mission_folder}')

    warnings = []
    try:
        with open(cfg_path, 'r') as f:
            data = json.load(f)
    except json.JSONDecodeError as e:
        warnings.append(f'mission_config.json corrupt: {e} — rebuilding')
        data = {
            'mission_folder': str(mission_folder),
            'mission_started_at': None,
            'sections': [],
        }

    # Reconcile sections with what's actually on disk.  We only ADD
    # entries we find — don't delete anything from the existing
    # sections list (it may include `deleted: true` entries the
    # coordinator wants to preserve for audit).
    on_disk = {}
    for section_dir in mission_folder.parent.glob('Section_*'):
        if section_dir.is_dir():
            on_disk[section_dir.name] = section_dir

    existing_names = {s.get('section_name') for s in data.get('sections', [])
                      if isinstance(s, dict)}
    # Strip any trailing _<status> tag from the on-disk dir name when
    # comparing — section_name is always the bare base name without
    # the post-completion completion tag.
    sections = data.get('sections', [])
    for dir_name, dir_path in on_disk.items():
        # Try to extract the bare Section_N_HHMMSS prefix (everything
        # before the first _complete / _partial / _gnss tag).
        bare = dir_name
        for marker in ('_complete', '_partial', '_gnss', '_aborted'):
            if marker in bare:
                bare = bare.split(marker, 1)[0]
                break
        if bare not in existing_names:
            warnings.append(
                f'unrecorded section folder on disk: {dir_name} — adding stub entry')
            sections.append({
                'section_name': bare,
                'section_folder': str(dir_path),
                'start_time': None,
                'end_time': datetime.fromtimestamp(
                    dir_path.stat().st_mtime).isoformat(),
                'completion_tag': 'recovered_via_ssh',
                'deleted': False,
            })

    finalized_at = datetime.now().isoformat()
    data.update({
        'sections': sections,
        'mission_finalized_at': finalized_at,
        'finalized_via': 'ssh_offline',
        'finalized_via_warnings': warnings,
    })
    atomic_write_json(cfg_path, data)
    return {
        'finalized': True,
        'mission_folder': str(mission_folder),
        'finalized_at': finalized_at,
        'section_count': len(sections),
        'warnings': warnings,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description='Robot-side standalone mission finaliser')
    parser.add_argument('--base-dir', default='/R_DATA',
                        help='Root data directory to search (default /R_DATA)')
    parser.add_argument('--mission-folder', default='',
                        help='Specific mission folder to finalize (overrides search)')
    args = parser.parse_args()

    base_dir = Path(args.base_dir)
    mission_folder = None
    if args.mission_folder:
        mission_folder = Path(args.mission_folder)
        if not mission_folder.exists():
            print(json.dumps({'finalized': False,
                              'reason': f'mission folder does not exist: {mission_folder}'}))
            return 3
    else:
        mission_folder = find_open_mission(base_dir)

    if mission_folder is None:
        print(json.dumps({'finalized': False,
                          'reason': 'no open mission found',
                          'base_dir': str(base_dir)}))
        return 0

    if not os.access(mission_folder, os.W_OK):
        print(json.dumps({'finalized': False,
                          'reason': f'mission folder not writable: {mission_folder}'}))
        return 3

    try:
        result = finalize_mission(mission_folder)
    except Exception as e:
        print(json.dumps({'finalized': False,
                          'reason': f'finalize failed: {e}',
                          'mission_folder': str(mission_folder)}))
        return 3

    print(json.dumps(result))
    return 0


if __name__ == '__main__':
    sys.exit(main())
