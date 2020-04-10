#!/usr/bin/env python3 

import rospy
import math
import tf.transformations as tr
from  tf.transformations import quaternion_from_euler
from  tf.transformations import euler_from_quaternion
import tf2_ros
import tf2_py as tf2
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from threading import Thread

def get_tf_transform(in_which_frame, what_frame, tf_buffer):
	try:
		trans = tf_buffer.lookup_transform(in_which_frame, what_frame,
		                                   rospy.Time(0),
		                                   rospy.Duration(4))
	except tf2.LookupException as ex:
		rospy.logwarn(ex)
		return None
	except tf2.ExtrapolationException as ex:
		rospy.logwarn(ex)
		return None
	return trans

def scan_matcher(x,y,z,q):
	return x,y,z,q

def rad(degrees):
	return degrees/180.0*math.pi

def deg(radians):
	return radians/math.pi*180.0

class MyThread(Thread):
	def __init__(self,tf_buffer):
		"""Инициализация потока"""
		Thread.__init__(self)
		self.tf_buffer = tf_buffer

	def run(self):
		rate = rospy.Rate(1.0)
		while not rospy.is_shutdown():
			trans = get_tf_transform('world', 'robot_pose', self.tf_buffer)
			if trans == None:
				continue
			x_got,y_got,z_got = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z 
			q_got = euler_from_quaternion((trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
			print("------------")
			print(x_got,y_got,z_got)
			print(deg(q_got[0]),deg(q_got[1]),deg(q_got[2]))
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('turtle_tf_listener')

	tf_buffer = tf2_ros.Buffer()
	tf_bc = tf2_ros.TransformBroadcaster()
	tf_listener = tf2_ros.TransformListener(tf_buffer)
	rate = rospy.Rate(10.0)
	i = 0
	x,y,z = 5,5,5
	q = (rad(45), rad(45), rad(45))
	dx_orig, dy_orig, dz_orig = 0,0,0
	dq_orig = (rad(90), rad(0), rad(0))
	
	t = MyThread(tf_buffer)
	t.start()
	while not rospy.is_shutdown():
		trans = TransformStamped()
		trans.header.seq = i
		i += 1
		trans.header.stamp = rospy.Time.now()
		trans.header.frame_id = 'odom'
		trans.child_frame_id = 'robot_pose'
		trans.transform.translation.x = x
		trans.transform.translation.y = y
		trans.transform.translation.z = z
		q_ = quaternion_from_euler(q[0], q[1], q[2])
		trans.transform.rotation = Quaternion(q_[0], q_[1], q_[2], q_[3])
		tf_bc.sendTransform(trans)

		dx_sm, dy_sm, dz_sm, dq_sm = scan_matcher(dx_orig,dy_orig,dz_orig, dq_orig)

		trans = TransformStamped()
		trans.header.seq = i
		i += 1
		trans.header.stamp = rospy.Time.now()
		trans.header.frame_id = 'world'
		trans.child_frame_id = 'odom'
		trans.transform.translation.x = dx_sm
		trans.transform.translation.y = dy_sm
		trans.transform.translation.z = dz_sm
		q_ = quaternion_from_euler(dq_sm[0], dq_sm[1], dq_sm[2])
		trans.transform.rotation = Quaternion(q_[0], q_[1], q_[2], q_[3])
		tf_bc.sendTransform(trans)

		

		rate.sleep()

'''import numpy as np

if __name__ == "__main__":
	v = np.array([1,1,1,0])
	u = np.random.rand(3)
	u = u/np.linalg.norm(u)
	u = np.array([1,0,0])
	a_2 = np.random.randn(1)
	a_2 = np.array([rad(22.5)])
	print(np.sin(a_2)*u)
	print(np.cos(a_2))
	q = np.concatenate((np.sin(a_2)*u, np.cos(a_2)))
	q_ = np.concatenate((np.sin(-a_2)*u, np.cos(a_2)))

	print(tr.quaternion_multiply(tr.quaternion_multiply(q,v),q_))
'''









