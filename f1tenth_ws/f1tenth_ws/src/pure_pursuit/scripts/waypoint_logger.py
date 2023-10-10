#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from datetime import datetime
from nav_msgs.msg import Odometry
import math
from rclpy.clock import Clock
import os

# create a new file
file_name = '/home/nvidia/' + "path" + '.csv'
file = open(file_name, 'w')
file.close()

class WaypointLoggerNode(Node):
	def __init__(self):
		super().__init__('WaypointLoggerNode')

		self.subscription_ = self.create_subscription(Odometry, '/pf/pose/odom', self.callback, 10)
		self.x = self.y = self.last_x = self.last_y = None
		self.inp = input("Press y/Y to enter the point: ")
		self.jk = Clock().now().nanoseconds/1000000000.0
		

	def logger(self):
		time = Clock().now().nanoseconds/1000000000.0
		#run the condition if time elapsed is more than 1 sec	
		if (time - self.jk > 0.5):
			self.jk = time
			if(self.inp in ['y', 'Y']): # add point
				with open(file_name, 'a') as file:
					file.write('%f, %f\n'%(self.x, self.y))
					print('Wrote: %f, %f\n'%(self.x, self.y))
					self.last_x = self.x
					self.last_y = self.y
			elif(self.inp in ['d', 'D']): # delete last line
				with open(file_name, "r") as f:
					lines=f.readlines()
					lines=lines[:-1] # deleting last line

				with open(file_name, 'w') as f:
					for i in lines:
						f.write(i)
					print("Successfully deleted the last point")

			elif(self.inp in ['q', 'Q']): # quit
				save = input("Do you want to save your file(y/n): ")
				if(save in ['n','N']):
					os.remove(file_name)
				exit()

	def callback(self, data):
		
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.logger()		

 
def main(args=None):
	rclpy.init(args=args)
	wp = WaypointLoggerNode()
	rclpy.spin(wp)
	rclpy.shutdown()


if __name__ == '__main__':
	main()
