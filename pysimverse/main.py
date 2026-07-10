# pip install pysimverse
# pip install keyboard
#
# python api
# https://github.com/Otr437/PYTHON_DRONE_SIMVERSE

from pysimverse import Drone
import socket
import select
import threading
import keyboard
import signal

# Global valiable
yaw = 0
pitch = 0
roll = 0
running = True
last_yaw = 0
dist = 2

# Ctrl-C handler
def signal_handler(sig, frame):
	global running
	#print("Press Ctrl-C")
	running = False

# UDP receiver
def udp():
	global yaw
	global pitch
	global roll
	global running

	print("Start udp")
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	sock.bind(('', 5005))

	while running:
		result = select.select([sock],[],[],1)
		#print(result[0], type(result[0]), len(result[0]))
		if (len(result[0]) != 0):
			line = result[0][0].recv(1024)
			if (type(line) is bytes):
				line=line.decode('utf-8')
			#print("line={}".format(line))
			yaw = float(line.split('y')[1])
			pitch = float(line.split('p')[1])
			roll = float(line.split('r')[1])
			#print("yaw={} pitch={} roll={}".format(yaw, pitch, roll))

# Keybord handler
def key():
	global running

	while running:
		if keyboard.is_pressed("esc"):
			running = False
		if keyboard.is_pressed("q"):
			running = False

signal.signal(signal.SIGINT, signal_handler)

drone = Drone()
drone.connect()
drone.take_off()

thread1 = threading.Thread(target=udp)
thread1.start()
thread2 = threading.Thread(target=key)
thread2.start()

while running:
	print("running={}".format(running))
	print("yaw={} pitch={} roll={}".format(yaw, pitch, roll))
	if last_yaw != yaw:
		delta_yaw = yaw - last_yaw
		drone.rotate(delta_yaw)
		last_yaw = yaw
	if pitch < -1:
		drone.move_forward(dist)
	if pitch > 1:
		drone.move_backward(dist)
	if roll < -1:
		drone.move_left(dist)
	if roll > 1:
		drone.move_right(dist)

thread1.join()
thread2.join()
drone.land()
