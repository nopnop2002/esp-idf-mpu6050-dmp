#!/usr/bin/env python3
#-*- encoding: utf-8 -*
#
# I used this as reference.
# https://www.kkaneko.jp/cc/3d/panda3dintro.html
#
# We need this library.
# python3 -m pip install panda3d
#
# pandagallery
# https://www.alice.org/pandagallery/Vehicles/index.html
from direct.showbase.ShowBase import ShowBase
from direct.showbase.Loader import Loader
from panda3d.core import Quat
from direct.task import Task
import socket
import select
import argparse

class HelloWorld(ShowBase):

	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.sock.bind(('', 5005))

		ShowBase.__init__(self)

		if (args.model == 'jet'):
			self.model = self.loader.loadModel("pandagallery/jet/jet.egg")
			#tex = loader.loadTexture("pandagallery/jet/maps/jet_Material_#6_CL.tif")
			tex = loader.loadTexture("pandagallery/jet/maps/wings_Material_#4_CL.tif")
			self.model.setTexture(tex, 1)
			self.model.setPos(0, 500, 0)
		elif (args.model == 'biplain'):
			self.model = self.loader.loadModel("pandagallery/Biplane/Biplane.egg")
			tex = loader.loadTexture("pandagallery/Biplane/maps/TextureMap.tif")
			self.model.setTexture(tex, 1)
			self.model.setPos(0, 200, 0)
		elif (args.model == '707'):
			self.model = self.loader.loadModel("pandagallery/Boeing707/Boeing707.egg")
			tex = loader.loadTexture("pandagallery/Boeing707/maps/TextureMap.tif")
			self.model.setTexture(tex, 1)
			self.model.setPos(0, 200, 0)
		elif (args.model == 'fa18'):
			self.model = self.loader.loadModel("pandagallery/NavyJet/NavyJet.egg")
			tex = loader.loadTexture("pandagallery/NavyJet/maps/TextureMap.tif")
			self.model.setTexture(tex, 1)
			self.model.setPos(0, 200, 0)
		self.model.reparentTo(self.render)
		# model color
		#self.model.setColorScale(0.0, 0.0, 0.0, 1.0)
		self.model.setScale(1, 1, 1)
		self.model.setQuat( Quat( 0, 1, 1, 1 ) )

		self.taskMgr.add(self.MotionTask, "MotionTask")


	def MotionTask(self, task):
		#print("MotionTask self.yaw={}".format(self.yaw));
		result = select.select([self.sock],[],[],1)
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
			self.model.setHpr(yaw, pitch, roll)
		
		return Task.cont

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('--model', choices=['jet', 'biplain', '707', 'fa18'], default='jet')
	args = parser.parse_args()
	print("args.model={}".format(args.model))

	app = HelloWorld()
	app.run()
