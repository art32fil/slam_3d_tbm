#!/usr/bin/env pyhton3
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import numpy as np
import numexpr as ne
import rospy

def cell_to_position(cell, scale):
	x = float((cell[0]+0.5)*scale)
	y = float((cell[1]+0.5)*scale)
	z = float((cell[2]+0.5)*scale)
	return Point(x,y,z)

class Cell:
	def __init__(self, a=0.05, b=0.05, ab=0.9):
		summ = a+b+ab
		self.a = a/summ
		self.b = b/summ
		self.ab = ab/summ

	def __add__(self, other):
		zero = self.a * other.b + self.b*other.a
		a = self.a*other.a + self.ab*other.a + self.a*other.ab
		b = self.b*other.b + self.ab*other.b + self.b*other.ab
		ab = self.ab*other.ab
		if (abs(zero +a+b+ab - 1) > 0.00001):
			print ("TBM cell merge is incorrect! or there are incorrect cells")
		return Cell(a,b,ab)

	def __iadd__(self, other):
		self = self + other
		return self

	def to_prob(self):
		return self.a + self.ab/2

def merge_cells(c1, c2):
	c1_a = c1[...,0]
	c1_b = c1[...,1]
	c1_ab = c1[...,2]
	c2_a = c2[...,0]
	c2_b = c2[...,1]
	c2_ab = c2[...,2]
	
	neqzero = ne.evaluate("1.0 - (c1_a*c2_b + c1_b*c2_a)")
	a = ne.evaluate("c1_a*c2_a + c1_ab*c2_a + c1_a*c2_ab")
	b = ne.evaluate("c1_b*c2_b + c1_ab*c2_b + c1_b*c2_ab")
	ab = ne.evaluate("c1_ab*c2_ab")
	answer = np.full_like(c1,0)
	answer[...,0] = ne.evaluate("a/neqzero")
	answer[...,1] = ne.evaluate("b/neqzero")
	answer[...,2] = ne.evaluate("ab/neqzero")
	return answer

def cell_to_prob(c):
	return c[...,0] + c[...,2]/2

class Map:
	def __init__(self, scale):
		self.Nx, self.Ny, self.Nz = 500, 500, 500
		#self.map = [[[0] * self.Nz for i in range(self.Ny)] for j in range(self.Nx)]
		self.map = np.zeros((self.Nx, self.Ny, self.Nz, 3), dtype=float)
		self.map[:,:,:,0] = 0.05
		self.map[:,:,:,1] = 0.05
		self.map[:,:,:,2] = 0.9

		self.scale = scale
		self.Zx, self.Zy, self.Zz = int(self.Nx/2), int(self.Ny/2), int(self.Nz/2)
		self.i = 0
	def update_cells(self, array, value):
		array_shifted = array + np.array([self.Zx, self.Zy, self.Zz])
		try:
			cells = self.map[array_shifted[:,0],array_shifted[:,1],array_shifted[:,2]]
			updated_cells = merge_cells(cells,value)
			self.map[array_shifted[:,0],array_shifted[:,1],array_shifted[:,2]] = updated_cells
		
		except IndexError:
			print("zhopa")
			return
	def to_marker(self):
		marker_msg = Marker()
		marker_msg.header.stamp = rospy.Time.now()
		marker_msg.header.frame_id = "world_world"
		marker_msg.ns = 'b'
		marker_msg.id = 0
		marker_msg.type = Marker.POINTS
		marker_msg.action = Marker.ADD
		marker_msg.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
		marker_msg.scale = Vector3(0.03,0.03,0.01)
		marker_msg.color = ColorRGBA(0,1,0,0.5)
		marker_msg.lifetime = rospy.Duration(0)
		marker_msg.points = []

		'''for i in range(self.Nx):
			for j in range(self.Ny):
				for k in range(self.Nz):
					if self.map[i][j][k] != 0:
						pt = cell_to_position((i-self.Zx,j-self.Zy,k-self.Zz), self.scale)
						marker_msg.points.append(pt)'''
		occ_points = np.transpose(np.array(np.where(self.map[:,:,:,0] > 0.93)))
		for p in occ_points:
			pt = cell_to_position((p[0] - self.Zx,p[1]-self.Zy,p[2]-self.Zz), self.scale)
			marker_msg.points.append(pt)
		
		return marker_msg
