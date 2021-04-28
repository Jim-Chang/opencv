import os
import cv2
import requests
from log import logging
from utils import im_nparr_2_bytes, im_bytes_2_nparr, im_concat

LOG_FOLDER = 'log'
LOG_IMG_FOLDER = 'log/img'
LOG_FILE = 'detect_log.csv'

LINE_NOTIRY_API = 'https://notify-api.line.me/api/notify'
LINE_NOTIFY_TOKEN = 'QnBIooUKaAKSsBNmMPQcpMlFom8MDjwPxvVmGeyyXEi'
LING_NOTIFY_HEADERS = {'Authorization': 'Bearer {}'.format(LINE_NOTIFY_TOKEN)}

DETECT_MSG = '我發現有人回家了'
DETECT_MSG_KNOWN = '，是{}'
DETECT_MSG_UNKNOWN = '，但有我不認識的人'
NO_DETECT_MSG = '我發現影片裡面沒有人'

MOTION_IP = 'http://192.168.68.58:7999'

def mk_log_dir_if_need():
    if not os.path.isdir(LOG_FOLDER):
        os.mkdir(LOG_FOLDER)

    if not os.path.isdir(LOG_IMG_FOLDER):
        os.mkdir(LOG_IMG_FOLDER)

def send_notify(msg: str, img: bytes = None):
    if img:
        requests.post(LINE_NOTIRY_API, headers=LING_NOTIFY_HEADERS, data={'message': msg}, files={'imageFile': img})
    else:
        requests.post(LINE_NOTIRY_API, headers=LING_NOTIFY_HEADERS, data={'message': msg})

# img => GBR
def send_notify_if_detect(results, img):
    if results:
        msg = DETECT_MSG + DETECT_MSG_KNOWN.format(', '.join(r.name for r in results if r.is_unknown is False))
        if any(r.is_unknown for r in results):
            msg += DETECT_MSG_UNKNOWN

        send_notify(msg, im_nparr_2_bytes(img))

# ExistRecord
def send_detected_notify_with_data_set(name_set, img_bdata_set, has_unknown, dry_run=False):
    if name_set or has_unknown:
        msg = DETECT_MSG

        if name_set:
            msg += DETECT_MSG_KNOWN.format(', '.join(n for n in name_set))

        if has_unknown:
            msg += DETECT_MSG_UNKNOWN

    else:
        msg = NO_DETECT_MSG

    if img_bdata_set:
        imgs = [im_bytes_2_nparr(bdata) for bdata in img_bdata_set]
        big_img = im_nparr_2_bytes(im_concat(imgs))

    else:
        big_img = None

    if dry_run:
        print(f'message: {msg}')
        print(f'img count: {len(img_bdata_set)}')

    else:
        send_notify(msg, big_img)

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
    if results and any(not r.is_unknown for r in results):
        disable_motion_detector()

def disable_motion_detector(camera_id=1, dry_run=False):
    if dry_run:
        print('我關閉動態偵測了')
        return

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
