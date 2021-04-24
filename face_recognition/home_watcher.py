from collections import defaultdict
from datetime import datetime, timedelta
import time
import os

from watcher import Watcher
from detector import FaceDetector
from reactor import mk_log_dir_if_need, write_to_log, send_notify_if_detect, save_img
from log import logging

CAM_URL = os.environ.get('CAM_URL', 'http://192.168.68.58:8081')
if CAM_URL.isnumeric():
    CAM_URL = int(CAM_URL)

INACTIVE_SECS = 30

# name: datetime
last_detect_history = defaultdict(lambda: None)

def _filter_last_detect(results, now):
    filtered_result = []

    for result in results:
        last_detect_at = last_detect_history[result.name]
        # 出現後 30 秒為不反應期
        if last_detect_at is None or now - last_detect_at > timedelta(seconds=INACTIVE_SECS):
            logging.info(f'detect face, face name: {result.name}')
            last_detect_history[result.name] = now
            filtered_result.append(result)
        else:
            logging.info(f'in inactive state, face name: {result.name}')

    return filtered_result

def detect_callback(results, img):
    now = datetime.now() + timedelta(hours=8)
    filtered_result = _filter_last_detect(results, now)

    save_img(filtered_result, now, img)
    write_to_log(filtered_result, now)
    send_notify_if_detect(filtered_result)

def main():
    mk_log_dir_if_need()

    watcher = Watcher(
        url=CAM_URL,
        detector=FaceDetector(),
        event_func=detect_callback,
        resize_factor=0.5
    )
    watcher.run()


if __name__ == '__main__':
    main()