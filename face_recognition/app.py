import threading
from flask import Flask, request, make_response
from reactor import send_notify
from file_watcher import start_watcher

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

    t = threading.Thread(target=start_watcher, args=(f'{date}/{filename}',))                                                              
    t.run()

    return make_response('', 200)