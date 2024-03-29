import os
from flask import Flask, request, make_response
from reactor import send_notify
from file_watcher import start_watcher

RESIZE_FACTOR = float(os.environ.get('RESIZE_FACTOR', 1))
VIDEO_SERVER_URL = os.environ.get('VIDEO_SERVER_URL', '')

app = Flask(__name__)

@app.route('/api/motion/record/new', methods=['POST'])
def on_new_record():
    '''
    {
        'filename': '/var/lib/motioneye/Camera1/2021-01-04/12-16-03.mp4',
    }
    '''
    data = request.json['filename'].split('/')
    camera, date, filename = data[4], data[5], data[6]
    send_notify(f'發現動靜！\n日期：{date}\n檔名：{filename}\n辨識中...')

    start_watcher(remote_file_name=f'{VIDEO_SERVER_URL}{date}/{filename}', resize_factor=RESIZE_FACTOR)

    return make_response('', 200)