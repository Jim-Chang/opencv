from flask import Flask, request, jsonify, render_template, Response

app = Flask(
    __name__,
    static_url_path='', 
    static_folder='static',
    template_folder='static',
)

@app.route("/", methods=['GET'])
def get_index_html():
    return render_template('index.html')

@app.route("/stream", methods=['GET'])
def get_stream_html():
    return render_template('stream.html')

@app.route("/api/direction", methods=['POST'])
def recieve_direction_cmd():
    data = request.json
    publisher = app.joystick_cmd_publisher
    publisher.pub_motor_cmd(data.get('speed', 0) or 0, data.get('diff', 0) or 0)
    return jsonify({'result': 'ok'})

@app.route('/api/stream')
def video_stream():
    return Response(app.image_receiver.gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/rec')
def set_rec_status():
    is_rec = request.args.get('status')

    publisher = app.joystick_cmd_publisher
    publisher.pub_rec_cmd(is_rec)
    return jsonify({'result': 'ok'})