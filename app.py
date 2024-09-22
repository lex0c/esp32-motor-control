from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
import socket
import json


ESP32_IP = "192.168.0.101"
ESP32_PORT = 8008

esp_socket = None


app = Flask(__name__)
CORS(app)


def send_command_to_esp32(command):
    try:
        esp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        esp_socket.connect((ESP32_IP, ESP32_PORT))

        msg = json.dumps(command) + '\n'

        esp_socket.sendall(msg.encode('utf-8'))

        esp_socket.shutdown(socket.SHUT_WR)
        esp_socket.close()
    except Exception as e:
        print(e)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/data', methods=['GET'])
def get_data():
    command = request.args.get('command')
    speed = request.args.get('speed')
    turn_speed = request.args.get('turn_speed')

    data = {
        'command': command,
        'speed': int(speed),
    }

    if command == 'left':
        data['command'] = 'turn'
        data['leftSpeed'] = 0 #int(turn_speed) * -1
        data['rightSpeed'] = int(turn_speed)

    if command == 'right':
        data['command'] = 'turn'
        data['leftSpeed'] = int(turn_speed)
        data['rightSpeed'] = 0 #int(turn_speed) * -1

    if data['speed'] == 0:
        data = { 'command': 'stop' }

    print(data)
    print(send_command_to_esp32(data))

    return jsonify(data), 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8001, debug=True)

