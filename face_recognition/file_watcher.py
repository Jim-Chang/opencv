from collections import defaultdict
from datetime import datetime, timedelta
import time
import os

from watcher import MTWatcher
from detector import FaceDetector
from reactor import mk_log_dir_if_need, write_to_log, send_notify_if_detect, save_img, disable_motion_detector_if_need
from log import logging

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
    pass
    # now = datetime.now() + timedelta(hours=8)
    # filtered_result = _filter_last_detect(results, now)

    # save_img(filtered_result, now, img)
    # write_to_log(filtered_result, now)
    # send_notify_if_detect(filtered_result, img)
    # disable_motion_detector_if_need(filtered_result)

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


if __name__ == '__main__':
    file_path = 'videos/18-24-13.mp4'
    main(file_path)