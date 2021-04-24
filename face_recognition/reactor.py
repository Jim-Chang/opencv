import os

LOG_FOLDER = 'log'
LOG_FILE = 'detect_log.csv'

def send_notify_if_detect(results):
    pass

def write_to_log(results, detect_at):
    if not os.path.isdir(LOG_FOLDER):
        os.mkdir(LOG_FOLDER)

    with open(f'{LOG_FOLDER}/{LOG_FILE}', 'a') as f:

        for result in results:
            text = f'{result.name}, {detect_at.strftime("%Y/%m/%d %H:%M:%S")}\n'
            f.write(text)