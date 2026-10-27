#!/usr/bin/env python3
"""Run micrus's bag_to_lerobot.py unchanged, with rosbags made tolerant of CRLF message definitions
(realsense2_camera_msgs/Metadata, see usrig_common/bag_io.py). Same arguments as bag_to_lerobot.py:

  python export/run_bag_to_lerobot.py -c lerobot_rig.yaml -o ~/datasets/us_v21 ~/datasets/staging

The converter is $BAG_TO_LEROBOT (scripts/env.sh).
"""
import os
import runpy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'src' / 'usrig_common'))
from usrig_common.bag_io import tolerate_crlf_msgdefs  # noqa: E402


def main():
    conv = os.environ.get('BAG_TO_LEROBOT')
    if not conv or not Path(conv).exists():
        sys.exit('BAG_TO_LEROBOT not set or missing (source scripts/env.sh)')
    tolerate_crlf_msgdefs()
    sys.argv = [conv] + sys.argv[1:]
    runpy.run_path(conv, run_name='__main__')


if __name__ == '__main__':
    main()
