import os
import cv2

LOG_FOLDER = 'log'
LOG_IMG_FOLDER = 'log/img'
LOG_FILE = 'detect_log.csv'

def mk_log_dir_if_need():
    if not os.path.isdir(LOG_FOLDER):
        os.mkdir(LOG_FOLDER)

    if not os.path.isdir(LOG_IMG_FOLDER):
        os.mkdir(LOG_IMG_FOLDER)

def send_notify_if_detect(results):
    pass

def write_to_log(results, detect_at):
    with open(f'{LOG_FOLDER}/{LOG_FILE}', 'a') as f:

        for result in results:
            text = f'{result.name}, {detect_at.strftime("%Y/%m/%d %H:%M:%S")}\n'
            f.write(text)

# img => RGB  
def save_img(results, detect_at, img):
    if results:
        filename = f'{detect_at.strftime("%Y%m%d-%H%M%S")}_' + '_'.join([r.name for r in results]) + '.jpg'
        cv2.imwrite(LOG_IMG_FOLDER+ '/' + filename, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))