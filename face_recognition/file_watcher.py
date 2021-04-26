from collections import defaultdict
from datetime import datetime, timedelta
import time
import os
import sys
from typing import NamedTuple, List

from watcher import MTWatcher
from detector import FaceDetector
from reactor import imencode_jpg, send_notify_with_exist_recs
from reactor import mk_log_dir_if_need, save_img, disable_motion_detector_if_need
from log import logging

VIDEO_FOLDER = 'videos'

# ExistRecords
exist_recs = []
exist_name_set = set()

# name: (MatchResult, jpg_img_bytes)
class ExistRecord(NamedTuple):
    matchs: List
    img: bytes

# MatchResult, np array
def detect_callback(results, img):
    filtered_result = []
    
    for result in results:
        # 只抓第一次出現
        if result.name not in exist_name_set:
            logging.info(f'detect face, face name: {result.name}')
            exist_name_set.add(result.name)
            filtered_result.append(result)

        else:
            logging.info(f'this face has existed, name: {result.name}')
    
    exist_recs.append(ExistRecord(
        filtered_result,
        imencode_jpg(img)
    ))

def main(file_path):
    mk_log_dir_if_need()

    watcher = MTWatcher(
        url=file_path,
        detector=FaceDetector(),
        event_func=detect_callback,
        resize_factor=0.5
    )

    _t = time.time()
    watcher.run()
    logging.info(f'Use {time.time() - _t} seconds')

    send_notify_with_exist_recs(exist_recs)

    matchs = []
    for r in exist_recs:
        matchs += r.matchs
    disable_motion_detector_if_need(matchs)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('please provide video file name')
    
    else:
        file_name = sys.argv[1]
        main(f'{VIDEO_FOLDER}/{file_name}')