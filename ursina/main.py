from ursina import *
import socket
import select
import argparse

# Update cube position
def update():
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
		print("yaw={} pitch={} roll={}".format(yaw, pitch, roll))

		cube.rotation_x = pitch
		cube.rotation_y = yaw
		cube.rotation_z = roll

# Initialize the socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', 5005))

parser = argparse.ArgumentParser()
parser.add_argument('--texture', help='texture', default="brick")
args = parser.parse_args()
print("args.texture={}".format(args.texture))

# Initialize the Ursina application
app = Ursina(size=(960, 640))

# Create a red cube at the center of the screen
# https://www.ursinaengine.org/api_reference.html#Texture
cube = Entity(model='cube', color=color.red, scale=2, texture=args.texture)
#cube = Entity(model='cube', color=color.red)
#cube = Entity(model='cube', color=color.red, texture="white_cube")
#cube = Entity(model='cube', color=color.red, texture="brick")
#cube = Entity(model='cube', color=color.red, texture="sky_sunset")
#cube = Entity(model='cube', color=color.red, texture="grass")

# Create a flat surface on the ground
#plane = Entity(model='plane', color=color.light_gray, scale=10, position=(0, 0, 0))

# Camera settings
camera.fov = 20 # Field of View
EditorCamera()

# Run the application
app.run()
