#!/usr/bin/env python3

import rospy
import rosbag
import sensor_msgs
from sensor_msgs.msg import PointCloud2 as pcl2
from sensor_msgs import point_cloud2 as pcl2_handler
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
import tf.transformations as tr
import numpy as np
from ros_numpy import point_cloud2
import math
from sensor_msgs.msg import ChannelFloat32

from bresenham import bresenhamline
from cloud_to_marker import cloud_to_marker
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA

class Cloud:
	def __init__(self, name="", values=np.array([[]])):
		self.name = name
		self.values = values

class Cell:
	def __init__(self, a, nota, both):
		summ = a+nota+both
		self.a = a/summ
		self.nota = nota/summ
		self.both = both/summ

def position_to_cell(pose, scale):
	i = int(math.floor(pose[0]/scale))
	j = int(math.floor(pose[1]/scale))
	k = int(math.floor(pose[2]/scale))
	return np.array([[i,j,k]])

def positions_to_cell(array, scale):
	return np.floor(array/scale)

def cell_to_position(cell, scale):
	x = float((cell[0]+0.5)*scale)
	y = float((cell[1]+0.5)*scale)
	z = float((cell[2]+0.5)*scale)
	return Point(x,y,z)

class Map:
	def __init__(self, scale):
		self.Nx, self.Ny, self.Nz = 200, 200, 200
		self.map = [[[0] * self.Nz for i in range(self.Ny)] for j in range(self.Nx)]
		self.scale = scale
		self.Zx, self.Zy, self.Zz = int(self.Nx/2), int(self.Ny/2), int(self.Nz/2)
	def update(self, point, value):
		if (point[0]+self.Zx >= self.Nx or point[0]+self.Zx < 0 or
		    point[1]+self.Zy >= self.Ny or point[1]+self.Zy < 0 or
		    point[2]+self.Zz >= self.Nz or point[2]+self.Zz < 0):
			print("zhopa")
			return
		self.map[int(point[0])+self.Zx][int(point[1])+self.Zy][int(point[2])+self.Zz] = value
	def update_cells(self, array, value):
		try:
			array_shifted = array.astype(int) + np.array([self.Zx, self.Zy, self.Zz])
			for p in array_shifted:
				self.map[p[0]][p[1]][p[2]] = value
		except IndexError:
			return
	def to_marker(self):
		marker_msg = Marker()
		marker_msg.header.stamp = rospy.Time.now()
		marker_msg.header.frame_id = "world"
		marker_msg.ns = 'b'
		marker_msg.id = 0
		marker_msg.type = Marker.POINTS
		marker_msg.action = Marker.ADD
		marker_msg.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
		marker_msg.scale = Vector3(0.03,0.03,0.01)
		marker_msg.color = ColorRGBA(0,1,0,0.5)
		marker_msg.lifetime = rospy.Duration(0)
		marker_msg.points = []

		for i in range(self.Nx):
			for j in range(self.Ny):
				for k in range(self.Nz):
					if self.map[i][j][k] != 0:
						pt = cell_to_position((i-self.Zx,j-self.Zy,k-self.Zz), self.scale)
						marker_msg.points.append(pt)
		return marker_msg

def get_pose_angle(transform):
	if not isinstance(transform, TransformStamped):
		rospy.logwarn("try to convert tansform of incorrect type.\nThe type is %s\nThe possible type is %s",
		              type(transform).__name__, "TransformStamped")
	pose = transform.transform.translation
	orientation = transform.transform.rotation
	p = np.array([pose.x, pose.y, pose.z])
	q = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
	return p,q

def getSE3_matrix(pose,quaternion):
	# pose = np.array([x, y, z])
	# quaternion = np.array([x y z w])
	g = tr.quaternion_matrix(q)
	g[0:3, -1] = p
	return g
	

def transformStamped_to_SE3(transform):
	if not isinstance(transform, TransformStamped):
		rospy.logwarn("try to convert tansform of incorrect type.\nThe type is %s\nThe possible type is %s",
		              type(transform).__name__, "TransformStamped")
	p,q = get_pose_angle(transform)
	g = getSE3_matrix(p,q)
	return g

def transform_cloud(SE3, new_frame, cloud):
	cloud_out = Cloud()
	cloud_out.name = new_frame
	cloud_out.values = np.matmul(SE3, cloud.values)
	return cloud_out

def cost_of_scan(grid_map, scan, robot_pose):
	free_points = bresenhamline(scan, robot_pose,-1).astype(int) + np.array([grid_map.Zx, grid_map.Zy, grid_map.Zz])
	occupied_points = scan.astype(int) + np.array([grid_map.Zx, grid_map.Zy, grid_map.Zz])
	accumulator = 0
	for fp in free_points:
		try:
			accumulator += abs(grid_map.map[fp[0]][fp[1]][fp[2]] - 0)
		except IndexError:
			continue
	for op in occupied_points:
		try:
			accumulator += abs(grid_map.map[fp[0]][fp[1]][fp[2]] - 1)
		except IndexError:
			continue
	return accumulator
		
def find_best_pose(grid_map, scan_meters, robot_pose_meters, robot_quaternion, scale, world_frame):
	
	final_pose = robot_pose_meters
	final_quaternion = robot_quaternion
	SE3 = getSE3_matrix(final_pose, final_quaternion)
	cloud_out = transform_cloud(SE3, world_frame, scan_meters)
	cloud_cells = positions_to_cell(np.transpose(cloud_out.values[0:-1]), scale)
	robot_cells = position_to_cell(robot_pose_meters, scale)
	min_cost = cost_of_scan(grid_map, cloud_cells, robot_cells)
	final_robot_cells = robot_cells
	final_cloud_cells = cloud_cells

	for i in range(10):
		new_pose = final_pose + 10*np.random.randn(3)
		new_quaternion = final_quaternion + 10*np.random.randn(1,4)
		SE3 = getSE3_matrix(new_pose, new_quaternion)
		cloud_out = transform_cloud(SE3, world_frame, scan_meters)
		cloud_cells = positions_to_cell(np.transpose(cloud_out.values[0:-1]), scale)
		
		robot_cells = position_to_cell(new_pose, scale)

		cost = cost_of_scan(grid_map, cloud_cells, robot_cells)
		if cost < min_cost:
			min_cost = cost
			final_pose = new_pose
			final_quaternion = new_quaternion
			final_robot_cells = robot_cells
			final_cloud_cells = cloud_cells
	return final_pose, final_quaternion, final_robot_cells, final_cloud_cells

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

if __name__ == '__main__':
	rospy.init_node("name_by_default")

	rospy.loginfo("begin")

	bag_name = str(rospy.get_param('~bag/file_path'))
	bag = rosbag.Bag(bag_name, 'r')
	topic_pcl2_name = str(rospy.get_param('~bag/topic_pcl'))

	odom_frame = str(rospy.get_param('~tf/odom_frame'))
	camera_frame = str(rospy.get_param('~tf/pcl_frame'))
	
	rospy.loginfo("before_for")

	tf_buffer = tf2_ros.Buffer()
	tf_bc = tf2_ros.TransformBroadcaster()
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	pub1 = rospy.Publisher("~/transormed_pc", Marker, queue_size=10)
	pub2 = rospy.Publisher("~/original_pc", Marker, queue_size=10)

	scale = float(rospy.get_param("~map/meters_per_cell"))
	
	r = rospy.Rate(1)
	m = Map(scale)
	for topic, msg, time in bag.read_messages(topics = [topic_pcl2_name, "/tf", "/tf_static"]):
		if rospy.is_shutdown():
			break
		if topic == "/tf" or topic == "/tf_static":
			transorms = msg
			for msg_tf in transorms.transforms:
        			tf_bc.sendTransform(msg_tf)
		elif topic == topic_pcl2_name:
			cloud = Cloud(msg.name, np.reshape(np.array(msg.values, dtype=np.float32),(4,-1),'F'))
			#pts = pcl2_handler.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
			pts = cloud.values
			print(pts.shape)
			trans = get_tf_transform(in_which_frame = odom_frame, what_frame = camera_frame,
			                      tf_buffer = tf_buffer)
			if (trans == None):
				continue
			p,q = get_pose_angle(trans)
			SE3 = getSE3_matrix(p,q)
			
			
			camera_pose_cell = position_to_cell(p, scale)
			print(camera_pose_cell)
			
			#cloud_out = transform_cloud(SE3, trans.header.frame_id, cloud)
			#pts = cloud_out.values
			print("1")
			#print(cloud_out.values.shape)
			#occupied_cells = np.transpose(positions_to_cell(pts[0:-1,:],scale))
			(best_camera_pose_meters,
			best_camera_quaternion,
			best_camera_cells,
			best_cloud_cells) = find_best_pose(m, cloud, p, q, scale, odom_frame)
			cloud_out = transform_cloud(getSE3_matrix(best_camera_pose_meters, best_camera_quaternion),
			                            odom_frame, cloud)
			line = bresenhamline(best_cloud_cells, best_camera_cells,-1)
			print("2")
			m.update_cells(line,0)
			m.update_cells(best_cloud_cells,1)
			print("3")
			'''for i in range(len(pts[0])):
				pt_cell = position_to_cell(pts[:,i], scale)
				line = bresenhamline(camera_pose_cell, pt_cell, -1)
				for p in line:
					m.update(p,0)
				m.update(line[-1],1)'''
				

			pub1.publish(cloud_to_marker(cloud_out,1,0,0))
			pub2.publish(m.to_marker())
			#pub2.publish(cloud_to_marker(cloud,0,1,0))
			#r.sleep()



	rospy.loginfo("after_for")
















