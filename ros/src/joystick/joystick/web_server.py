from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route("/api/direction", methods=['POST'])
def recieve_direction_cmd():
    data = request.json
    publisher = app.joystick_cmd_publisher
    publisher.pub_cmd(data['speed'], data['diff'])
    return jsonify({'result': 'ok'})