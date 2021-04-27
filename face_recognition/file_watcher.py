import argparse
from collections import defaultdict
from datetime import datetime, timedelta
import time
import os
import sys
from typing import NamedTuple, List

from watcher import MTWatcher
from detector import FaceDetector, MatchResult
from utils import im_nparr_2_bytes
from reactor import send_notify_with_data_set, disable_motion_detector, mk_log_dir_if_need
from log import logging

VIDEO_FOLDER = 'videos'

# ExistRecords
exist_recs = []

# name: (MatchResult, jpg_img_bytes)
class ExistRecord(NamedTuple):
    matchs: List[MatchResult]
    img: bytes

# MatchResults, np array
def _detect_callback(results: List[MatchResult], img):
    exist_recs.append(ExistRecord(
        results,
        im_nparr_2_bytes(img)
    ))

def _filter_recs(recs: List[ExistRecord]):
    name_set = set()
    img_set = set()
    has_match = False
    has_unknown = False

    for rec in recs:
        for m in rec.matchs:
            # 已知，不重複加入
            if m.is_unknown is False and m.name not in name_set:
                name_set.add(m.name)
                img_set.add(rec.img)
                has_match = True

            # 未知，一率加入
            elif m.is_unknown is True:
                img_set.add(rec.img)
                has_unknown = True

    return name_set, img_set, has_match, has_unknown


def _arg_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', type=str, help='image file')
    parser.add_argument('-d', '--dryrun', action='store_true', help='dry run')
    return parser.parse_args()


def main():
    args = _arg_parse()

    file_path = f'{VIDEO_FOLDER}/{args.file}'
    dryrun = args.dryrun

    mk_log_dir_if_need()

    watcher = MTWatcher(
        url=file_path,
        detector=FaceDetector(),
        thread_num=3,
        event_func=_detect_callback,
        resize_factor=0.5
    )

    _t = time.time()
    watcher.run()
    logging.info(f'Use {time.time() - _t} seconds')

    name_set, img_set, has_match, has_unknown = _filter_recs(exist_recs)
    send_notify_with_data_set(name_set, img_set, has_unknown, dry_run=dryrun)

    if has_match:
        disable_motion_detector(dry_run=dryrun)


if __name__ == '__main__':
    main()
