import argparse
from collections import defaultdict
from datetime import datetime, timedelta
import time
import os
import sys
from typing import NamedTuple, List

from watcher import MTWatcher
from detector import FaceDetector, MatchResult
from utils import im_nparr_2_bytes, load_meta_data
from reactor import send_detected_notify_with_data_set, disable_motion_detector
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
    has_unknown = False

    for rec in recs:
        for m in rec.matchs:
            # 已知，不重複加入
            if m.is_unknown is False and m.name not in name_set:
                name_set.add(m.name)
                img_set.add(rec.img)

            # 未知，一率加入
            elif m.is_unknown is True:
                img_set.add(rec.img)
                has_unknown = True

    return name_set, img_set, has_unknown


def _arg_parse():
    parser = argparse.ArgumentParser()
    parser.add_argument('file_name', type=str, help='image file')
    parser.add_argument('-d', '--dryrun', action='store_true', help='dry run')
    return parser.parse_args()


def main(file_name, dryrun=False):
    file_path = f'{VIDEO_FOLDER}/{file_name}'

    if dryrun:
        logging.warning('Run in DRYRUN mode')

    meta_data = load_meta_data()
    
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

    name_set, img_set, has_unknown = _filter_recs(exist_recs)

    nickname_set = {meta_data.get_nickname(n) for n in name_set}
    send_detected_notify_with_data_set(nickname_set, img_set, has_unknown, dry_run=dryrun)

    authorize_name_set = meta_data.get_authorize_name_set()
    if len(name_set.intersection(authorize_name_set)) > 0:
        disable_motion_detector(dry_run=dryrun)


if __name__ == '__main__':
    args = _arg_parse()
    main(args.file_name, args.dryrun)
