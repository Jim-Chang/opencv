from flask import Flask, request, jsonify, render_template

app = Flask(
    __name__,
    static_url_path='', 
    static_folder='static',
    template_folder='static',
)

@app.route("/", methods=['GET'])
def get_index_html():
    return render_template('index.html')

@app.route("/api/direction", methods=['POST'])
def recieve_direction_cmd():
    data = request.json
    publisher = app.joystick_cmd_publisher
    publisher.pub_cmd(data['speed'], data['diff'])
    return jsonify({'result': 'ok'})