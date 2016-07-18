#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from threading import Thread
import time
import csv
import binascii
import pygame
import atexit
from pygame.locals import *



class App:

	def __init__(self):
		rospy.init_node('keyboard_controller', anonymous=True)
		self.cmd_pub = rospy.Publisher('user_command', TwistStamped, queue_size=10)
		pygame.init()
		screen = pygame.display.set_mode((468, 60))
		pygame.display.set_caption('keyboard_controller')
		self.run = True
		self.lState = 1
		self.rState = 1
		self.fState = 1
		self.bState = 1
		self.flState = 1
		self.frState = 1
		self.blState = 1
		self.brState = 1
		
		print "Initialised..."
		while not rospy.is_shutdown() and self.run:
			for event in pygame.event.get():
				keys = pygame.key.get_pressed()
				if event.type == pygame.KEYUP:
					self.stop(event)
				elif event.type == pygame.KEYDOWN:
					if keys[K_w] & keys[K_d]:
						self.forwardRight()
					elif keys[K_w] & keys[K_a]:
						self.forwardLeft()
					elif keys[K_s] & keys[K_d]:
						self.backRight()
					elif keys[K_s] & keys[K_a]:
						self.backLeft()
					elif event.key == pygame.K_w: 
						self.forward()
					elif event.key == pygame.K_a: 
						self.left()
					elif event.key == pygame.K_s: 
						self.backward()
					elif event.key == pygame.K_d: 
						self.right()
				
		return


	def stop(self,event):
		if (event.key == pygame.K_w)|(event.key == pygame.K_a)|(event.key == pygame.K_s)|(event.key == pygame.K_d):
			print 'stop'

			self.fState = 1
			self.lState = 1
			self.rState = 1
			self.bState = 1
			self.flState = 1
			self.frState = 1
			self.blState = 1
			self.brState = 1

			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()
			cmd.twist.linear.x = 0
			cmd.twist.angular.z = 0
			self.cmd_pub.publish(cmd)
			time.sleep(.1)

		return    

	def forward(self):
		#ensure command is only called once
		
		if self.fState == 1:
			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()
			cmd.twist.linear.x = 0.5
			cmd.twist.angular.z = 0
			self.cmd_pub.publish(cmd)
			print "forward"
			self.fState = 0
			time.sleep(.1)
		return    

	def backward(self):
		#ensure command is only called once
		
		if self.bState == 1:
			print "Back"
			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()
			cmd.twist.linear.x = -0.5
			cmd.twist.angular.z = 0
			self.cmd_pub.publish(cmd)
			self.bState = 0
			time.sleep(.1)
		return    

	def left(self):
		#ensure command is only called once
		
		if self.lState == 1:
			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()
			cmd.twist.linear.x = 0
			cmd.twist.angular.z = 0.5
			self.cmd_pub.publish(cmd)
			print "left"
			self.lState = 0
			time.sleep(.1)
		return  

	def right(self):
		#ensure command is only called once
		
		if self.rState == 1:
			print "right"
			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()
			cmd.twist.linear.x = 0
			cmd.twist.angular.z = -0.5
			self.cmd_pub.publish(cmd)
			self.rState = 0
			time.sleep(.1)
		return    

	def forwardRight(self):
		#ensure command is only called once
		
		if self.frState == 1:
			print "forward right"
			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()
			cmd.twist.linear.x = 0.5
			cmd.twist.angular.z = -0.5
			self.cmd_pub.publish(cmd)
			self.frState = 0
			time.sleep(.1)
		return    
	def forwardLeft(self):
		#ensure command is only called once
		if self.flState == 1:
			print "forward left"
			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()
			cmd.twist.linear.x = .50
			cmd.twist.angular.z = 0.5
			self.cmd_pub.publish(cmd)
			self.flState = 0
			time.sleep(.1)
		return
	def backRight(self):
		#ensure command is only called once
		
		if self.brState == 1:
			print "back right"
			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()        	
			cmd.twist.linear.x = -.5
			cmd.twist.angular.z = -0.5
			self.cmd_pub.publish(cmd)
			self.brState = 0
			time.sleep(.1)
		return    
	def backLeft(self):
		#ensure command is only called once
		
		if self.blState == 1:
			print "back left"
			cmd = TwistStamped()
			cmd.header.stamp = rospy.get_rostime()
			self.cmd_pub.publish(cmd)
			cmd.twist.linear.x = -.5
			cmd.twist.angular.z = 0.5
			self.cmd_pub.publish(cmd)
			self.blState = 0
			time.sleep(.1)
		return    

if __name__ == "__main__":
	App()

