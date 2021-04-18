from collections import defaultdict
import time
from datetime import datetime, timedelta

import os

from webcam import Webcam
from detector import FaceDetector

CAM_URL = os.environ.get('CAM_URL', 'http://192.168.68.58:8081/')

INACTIVE_SECS = 30

# name: datetime
last_detect_history = defaultdict(lambda: None)

def send_notify_if_detect(results):
    pass

def write_to_log(results):
    if results:
        with open('detect_log.csv', 'a') as f:
            now = datetime.now()

            for result in results:
                last_detect_at = last_detect_history[result.name]
                # 出現後 30 秒為不反應期
                if last_detect_at is None or now - last_detect_at > timedelta(seconds=INACTIVE_SECS):
                    last_detect_history[result.name] = now
                    text = f'{result.name}, {now.strftime("%Y/%m/%d %H:%M:%S")}'
                    f.write(text)
                    print(f'write to log: {text}')
                
                else:
                    print(f'in inactive state, face name: {result.name}')



def detect_callback(results):
    write_to_log(results)
    send_notify_if_detect(results)

def main():
    webcam = Webcam(
        url=CAM_URL,
        detector=FaceDetector(),
        event_func=detect_callback
    )
    webcam.run()


if __name__ == '__main__':
    main()