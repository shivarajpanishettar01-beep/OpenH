#!/usr/bin/env python3
"""LeRobot's own v2.1 -> v3.0 converter (unchanged), with one data parquet and one video file per camera per episode.

LeRobot v3.0 packs episodes into size-capped files. Its converter starts a new file when the next episode would
reach the cap and the current file already holds one, so a cap of 0 MB gives one episode per file. The cap is also
written to info.json, where DatasetInfo refuses 0 (it is only used when episodes are appended later), so info.json
gets LeRobot's default caps while the data and video conversion run with 0.

  python convert_v30.py lerobot.scripts.convert_dataset_v21_to_v30 --repo-id local/NAME --root DIR

Called by openh_export.py --files per-episode (the module name is the one it found in this LeRobot install).
"""
import argparse
import importlib
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('module', help="LeRobot's converter module")
    ap.add_argument('--repo-id', required=True)
    ap.add_argument('--root', required=True)
    a = ap.parse_args()
    m = importlib.import_module(a.module)
    need = ('convert_dataset', 'convert_info', 'DEFAULT_DATA_FILE_SIZE_IN_MB', 'DEFAULT_VIDEO_FILE_SIZE_IN_MB')
    missing = [n for n in need if not hasattr(m, n)]
    if missing:
        sys.exit(f'{a.module} has no {missing}: this LeRobot version changed the converter; '
                 'use openh_export.py --files packed')
    info = m.convert_info

    def convert_info(root, new_root, data_mb, video_mb):
        return info(root, new_root, data_mb or m.DEFAULT_DATA_FILE_SIZE_IN_MB, video_mb or m.DEFAULT_VIDEO_FILE_SIZE_IN_MB)

    m.convert_info = convert_info
    if hasattr(m, 'init_logging'):
        m.init_logging()
    m.convert_dataset(repo_id=a.repo_id, root=a.root, push_to_hub=False, data_file_size_in_mb=0, video_file_size_in_mb=0)


if __name__ == '__main__':
    main()
