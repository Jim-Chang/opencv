import os
import cv2
import requests
from log import logging

LOG_FOLDER = 'log'
LOG_IMG_FOLDER = 'log/img'
LOG_FILE = 'detect_log.csv'

LINE_NOTIRY_API = 'https://notify-api.line.me/api/notify'
LINE_NOTIFY_TOKEN = 'QnBIooUKaAKSsBNmMPQcpMlFom8MDjwPxvVmGeyyXEi'
LING_NOTIFY_HEADERS = {'Authorization': 'Bearer {}'.format(LINE_NOTIFY_TOKEN)}

DETECT_MSG = '我發現有人回家了'
DETECT_MSG_KNOWN = '，是{}'
DETECT_MSG_UNKNOWN = '\n但有我不認識的人'

MOTION_IP = 'http://192.168.68.58:7999'

def mk_log_dir_if_need():
    if not os.path.isdir(LOG_FOLDER):
        os.mkdir(LOG_FOLDER)

    if not os.path.isdir(LOG_IMG_FOLDER):
        os.mkdir(LOG_IMG_FOLDER)

def imencode_jpg(img):
    return cv2.imencode('.jpg', img)[1].tobytes()

# img => GBR
def send_notify_if_detect(results, img):
    if results:
        msg = DETECT_MSG + DETECT_MSG_KNOWN.format(', '.join(r.name for r in results if r.unknown is False))
        if any(r.unknown for r in results):
            msg += DETECT_MSG_UNKNOWN

        data = {'message': msg}
        files = {'imageFile': cv2.imencode('.jpg', img)[1].tobytes()}

        requests.post(LINE_NOTIRY_API, headers=LING_NOTIFY_HEADERS, data=data, files=files)

# ExistRecord
def send_notify_with_exist_recs(exist_recs):
    for rec in exist_recs:
        msg = DETECT_MSG + DETECT_MSG_KNOWN.format(', '.join(m.name for m in rec.matchs if m.unknown is False))
        if any(m.unknown for m in rec.matchs):
            msg += DETECT_MSG_UNKNOWN

        data = {'message': msg}
        files = {'imageFile': rec.img}

        requests.post(LINE_NOTIRY_API, headers=LING_NOTIFY_HEADERS, data=data, files=files)

def write_to_log(results, detect_at):
    if results:
        with open(f'{LOG_FOLDER}/{LOG_FILE}', 'a') as f:
            for result in results:
                text = f'{result.name}, {detect_at.strftime("%Y/%m/%d %H:%M:%S")}\n'
                f.write(text)

# img => GBR  
def save_img(results, detect_at, img):
    if results:
        filename = f'{detect_at.strftime("%Y%m%d-%H%M%S")}_' + '_'.join([r.name for r in results]) + '.jpg'
        cv2.imwrite(LOG_IMG_FOLDER+ '/' + filename, img)

def disable_motion_detector_if_need(results, camera_id=1):        
    if results and any(not r.unknown for r in results):
        if _get_motion_detector_status(camera_id) is False:
            return
            
        # 'http://192.168.68.58:7999/0/detection/pause'
        try:
            r = requests.get(f'{MOTION_IP}/{camera_id}/detection/pause')
            # Camera 1 Detection paused\nDone \n'
            if 'paused' in r.text:
                result = True
            else:
                logging.warning('[Motion] set detection stop fail: {}'.format(r.text))
                result = False

        except Exception:
            logging.error('[Motion] motion server connect fail', exc_info=True)
            result = False

        requests.post(
            LINE_NOTIRY_API,
            headers=LING_NOTIFY_HEADERS,
            data={'message': '我關閉動態偵測了' if result else '關閉偵測失敗了！'}
        )

def _get_motion_detector_status(camera_id=1):
    # 'http://192.168.68.58:7999/0/detection/status'
    try:
        r = requests.get(f'{MOTION_IP}/{camera_id}/detection/status')
        # Camera 1 Detection status ACTIVE \n
        # Camera 1 Detection status PAUSE \n
        return 'ACTIVE' in r.text
    except Exception:
        logging.error('[Motion] motion server connect fail', exc_info=True)
        return False
