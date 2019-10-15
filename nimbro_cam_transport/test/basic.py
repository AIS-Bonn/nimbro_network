#!/usr/bin/env python

# Test driver for basic test

from __future__ import print_function

import sys
import unittest
import time
import os
import math

import rospy
import rospy.client

import rospkg

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import cv2

rospack = rospkg.RosPack()

## A sample python unit test
class BasicTest(unittest.TestCase):
	def handle_reset(self):
		self.encoded = []
		self.image_rx = []

	def handle_encoded(self, msg):
		self.encoded.append(msg)

	def handle_image(self, msg):
		self.image_rx.append(msg)

	def test_transmission(self):
		bridge = CvBridge()
		self.handle_reset()

		pub = rospy.Publisher('/image_tx', Image, queue_size=20)
		sub_encoded = rospy.Subscriber('/image_encoded', CompressedImage, self.handle_encoded, queue_size=20)
		sub = rospy.Subscriber('/image_rx', Image, self.handle_image, queue_size=20)

		# Wait for subscriber
		start_time = time.time()
		while pub.get_num_connections() == 0 or sub_encoded.get_num_connections() == 0 or sub.get_num_connections() == 0:
			if time.time() - start_time > 10.0:
				if pub.get_num_connections() == 0:
					self.fail("Nobody subscribed on /image_tx")
				if sub_encoded.get_num_connections() == 0:
					self.fail("Nobody published on /image_encoded")
				if sub.get_num_connections() == 0:
					self.fail("Nobody published on /image_rx")

				return

			time.sleep(0.1)

		# Feed input
		image_tx = []
		for i in range(15):
			img_tx = np.zeros((480,640,3), dtype=np.uint8)

			cv2.putText(img_tx, '{:02d}'.format(i), (0,480), cv2.FONT_HERSHEY_SIMPLEX, 480/30, (255,255,255))
			image_tx.append(img_tx)

			pub.publish(bridge.cv2_to_imgmsg(img_tx, "bgr8"))

			if i == 0:
				time.sleep(2)
			else:
				time.sleep(0.2)

		start_time = time.time()
		while len(self.encoded) < 15:
			if time.time() - start_time > 8.0:
				self.fail("Did not get enough encoded frames (only {})".format(len(self.encoded)))
				return

			time.sleep(0.1)

		start_time = time.time()
		while len(self.image_rx) < 15:
			if time.time() - start_time > 8.0:
				self.fail("Did not get enough decoded frames (only {})".format(len(self.image_rx)))
				return

			time.sleep(0.1)

		for tx, rx in zip(image_tx, self.image_rx):
			rx = bridge.imgmsg_to_cv2(rx, "bgr8")

			tx_small = cv2.resize(tx, dsize=(0,0), fx=0.1, fy=0.1)
			rx_small = cv2.resize(rx, dsize=(0,0), fx=0.1, fy=0.1)

			diff = tx_small.astype(np.float) - rx_small.astype(np.float)
			diff = diff**2

			error = np.sum(diff) / diff.size

			if error > 0.0:
				cv2.imwrite('/tmp/tx.png', tx)
				cv2.imwrite('/tmp/rx.png', rx)
				self.fail("Image deviation by {} (I received {}/{} frames)".format(error, len(self.encoded), len(self.image_rx)))
				return

if __name__ == '__main__':
	rospy.init_node('basic_test')

	import rostest
	rostest.rosrun('nimbro_cam_transport', 'basic', BasicTest)

	time.sleep(1)
