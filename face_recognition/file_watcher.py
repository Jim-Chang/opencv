import argparse
from collections import defaultdict
from datetime import datetime, timedelta
import time
import os
import sys
from typing import List
from dataclasses import dataclass

from watcher import MTWatcher, Watcher
from detector import FaceDetector, MatchResult
from utils import im_nparr_2_bytes, load_meta_data
from reactor import send_detected_notify_with_data_set, disable_motion_detector
from log import logging

VIDEO_FOLDER = 'videos'


@dataclass
class ExistRecord:
    matchs: List[MatchResult]
    img: bytes


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
    parser.add_argument('-r', '--resize', type=float, help='resize factor')
    parser.add_argument('-a', '--algo', type=str, help='algorithm')
    return parser.parse_args()


def start_watcher(local_file_name=None, remote_file_name=None, dryrun=False, resize_factor=1, algo=None):
    if local_file_name:
        file_path = f'{VIDEO_FOLDER}/{local_file_name}'

    if remote_file_name:
        file_path = remote_file_name

    WatcherHandler(file_path, dryrun, resize_factor, algo).start()


class WatcherHandler:

    def __init__(self, file_path, dryrun=False, resize_factor=1, algo=None):
        self.file_path = file_path
        self.dryrun = dryrun
        self.resize_factor = resize_factor
        self.algo = algo

        if self.dryrun:
            logging.warning('Run in DRYRUN mode')

        # ExistRecords
        self.exist_recs = []
        self.matchs_name_set_buf = set()

        # MetaData
        self.meta_data = load_meta_data()
        self.authorize_name_set = self.meta_data.get_authorize_name_set()

    def start(self):
        if self.algo == 'mediapipe':
            logging.info('use mediapipe')
            watcher = Watcher(
                url=self.file_path,
                detector=FaceDetector(detect_by='mediapipe'),
                event_func=self._detect_callback,
                is_need_force_stop_func=self._has_detect_authorize_face,
                resize_factor=1,
                # show_window=True
            )

        else:
            logging.info('use face_recognition')
            watcher = MTWatcher(
                url=self.file_path,
                detector=FaceDetector(detect_by='face_recognition'),
                thread_num=4,
                event_func=self._detect_callback,
                is_need_force_stop_func=self._has_detect_authorize_face,
                resize_factor=self.resize_factor,
            )

        
        _t = time.time()
        watcher.run()
        logging.info(f'Use {time.time() - _t} seconds')

        name_set, img_set, has_unknown = self._filter_recs(self.exist_recs)

        nickname_set = {self.meta_data.get_nickname(n) for n in name_set}
        send_detected_notify_with_data_set(nickname_set, img_set, has_unknown, dry_run=self.dryrun)

        if len(name_set.intersection(self.authorize_name_set)) > 0:
            disable_motion_detector(dry_run=self.dryrun)


    # MatchResults, np array
    def _detect_callback(self, results: List[MatchResult], img):
        self.exist_recs.append(ExistRecord(
            results,
            im_nparr_2_bytes(img)
        ))
        self.matchs_name_set_buf.update([r.name for r in results])


    # 回傳是否有偵測到授權的臉
    def _has_detect_authorize_face(self) -> bool:
        return len(self.matchs_name_set_buf.intersection(self.authorize_name_set)) > 0


    @staticmethod
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


if __name__ == '__main__':
    args = _arg_parse()

    param = {
        'local_file_name': args.file_name,
        'dryrun': args.dryrun,
        'algo': args.algo,
    }

    if args.resize:
        param['resize_factor'] = args.resize

    start_watcher(**param)
