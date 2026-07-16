#!/usr/bin/env python3
"""TRON2 Web Walk — Flask + rospy inside Docker. Access: http://T14_IP:8080"""
import rospy
from flask import Flask
from std_msgs.msg import String
from geometry_msgs.msg import Twist

app = Flask(__name__)
rospy.init_node("web_walk", anonymous=True)
mode_pub = rospy.Publisher("/tron2_controller/set_mode", String, queue_size=1)
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

# Switch to WALK on first access
mode_pub.publish(String("WALK"))

SPEEDS = {
    'fw':   [0.3, 0, 0],
    'bw':   [-0.3, 0, 0],
    'left': [0, 0.2, 0],
    'right':[0, -0.2, 0],
    'tl':   [0, 0, 0.5],
    'tr':   [0, 0, -0.5],
    'stop': [0, 0, 0],
}

@app.route('/cmd/<action>', methods=['POST'])
def cmd(action):
    if action in SPEEDS:
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = SPEEDS[action]
        cmd_pub.publish(t)
        return 'ok'
    return 'error'

HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>TRON2 Walk</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:system-ui;background:#1a1a2e;color:#eee;display:flex;justify-content:center;padding:10px}
.w{max-width:400px;width:100%}
h1{text-align:center;margin:10px 0;color:#256BEB;font-size:20px}
.c{background:#16213e;border-radius:10px;padding:12px;margin:6px 0}
button{padding:22px 15px;border:none;border-radius:8px;font-size:16px;cursor:pointer;font-weight:700;min-width:60px}
button:active{transform:scale(0.95)}
.r{display:flex;gap:6px;justify-content:center;flex-wrap:wrap}
.fw{background:#256BEB;color:#fff}
.bw{background:#c0392b;color:#fff}
.lr{background:#533483;color:#fff}
.turn{background:#f39c12;color:#000}
.stop{background:#c0392b;color:#fff;width:100%}
.st{text-align:center;font-size:12px;color:#888;padding:4px}
</style></head><body><div class="w">
<h1>TRON2 Walk</h1>
<div class="c">
<div class="r"><button class="fw" onclick="S('fw')">Forward</button></div>
<div class="r" style="margin-top:6px">
<button class="lr" onclick="S('left')">Left</button>
<button class="stop" style="width:80px" onclick="S('stop')">Stop</button>
<button class="lr" onclick="S('right')">Right</button>
</div>
<div class="r" style="margin-top:6px"><button class="bw" onclick="S('bw')">Back</button></div>
<div class="r" style="margin-top:10px">
<button class="turn" onclick="S('tl')">TurnL</button>
<button class="turn" onclick="S('tr')">TurnR</button>
</div>
</div>
<div class="st" id="s">Ready</div>
</div>
<script>
async function S(cmd){document.getElementById('s').textContent=cmd;try{let r=await fetch('/cmd/'+cmd,{method:'POST'});document.getElementById('s').textContent=r.ok?'OK':'ERR'}catch(e){document.getElementById('s').textContent='ERR'}}
</script></body></html>"""

@app.route('/')
def index():
    return HTML

if __name__ == '__main__':
    import os
    ip = os.popen('hostname -I').read().strip().split()[0]
    print(f"Web Walk: http://{ip}:8080")
    app.run(host='0.0.0.0', port=8080, debug=False)
