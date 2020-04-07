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
from visualization_msgs.msg import Marker
from map import Map
from map import cell_to_prob
from map import merge_cells

from bresenham import bresenhamline
from cloud_to_marker import cloud_to_marker


class Cloud:
	def __init__(self, name="", values=np.array([[]])):
		self.name = name
		self.values = values

def position_to_cell(pose, scale):
	i = int(math.floor(pose[0]/scale))
	j = int(math.floor(pose[1]/scale))
	k = int(math.floor(pose[2]/scale))
	return np.array([[i,j,k]])

def positions_to_cell(array, scale):
	return np.floor(array/scale).astype(int)

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
	g = tr.quaternion_matrix(quaternion)
	g[0:3, -1] = pose
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
	occupied_points = scan.astype(int) + np.array([grid_map.Zx, grid_map.Zy, grid_map.Zz])
	occ_cells = grid_map.map[occupied_points[:,0],occupied_points[:,1],occupied_points[:,2]]
	value = np.sum(np.abs(cell_to_prob(merge_cells(occ_cells, np.array([0.5,0,0.5])))-1))

	return value#accumulator
		
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
	final_pose_delta = np.array([0.0, 0.0, 0.0])
	final_quaternion_delta = np.array([0.0, 0.0, 0.0, 1.0])

	for i in range(1000):
		delta_pose = 2*scale*np.random.randn(3)
		new_pose = final_pose + delta_pose
		theta_2 = 0.1*np.random.randn(1)
		delta_quaternion = np.concatenate((np.sin(theta_2)*np.random.rand(3), np.cos(theta_2))) 
		new_quaternion = tr.quaternion_multiply(final_quaternion, delta_quaternion)
		new_SE3 = getSE3_matrix(new_pose, new_quaternion)
		#print(SE3 - new_SE3)
		#print("before transform_cloud")
		#t = rospy.Time.now()
		cloud_out = transform_cloud(new_SE3, world_frame, scan_meters)
		#print("transform_cloud executes: ", rospy.Time.now() - t)
		#print("after transform_cloud")
		cloud_cells = positions_to_cell(np.transpose(cloud_out.values[0:-1]), scale)
		
		robot_cells = position_to_cell(new_pose, scale)

		#print("before cost_of_scan")
		#t = rospy.Time.now()
		cost = cost_of_scan(grid_map, cloud_cells, robot_cells)
		print("min_cost: ", min_cost, ", cost: ", cost)
		#print("cost_of_scan executes:    ", rospy.Time.now() - t)
		#print("after cost_of_scan")
		if cost < min_cost:
			min_cost = cost
			final_pose = new_pose
			final_quaternion = new_quaternion
			final_robot_cells = robot_cells
			final_cloud_cells = cloud_cells
			final_pose_delta = delta_pose
			final_quaternion_delta = delta_quaternion
	return final_pose, final_quaternion, final_robot_cells, final_cloud_cells, final_pose_delta, final_quaternion_delta

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

def time_to_s_ms_ns(t):
	t = t.to_nsec()
	secs = int(t/1000000000)
	msecs = int(t/1000000) - 1000*secs
	mksecs = int(t/1000) - 1000*msecs - 1000000*secs
	nsecs = t - 1000*mksecs - 1000000*msecs - 1000000000*secs
	return "%ss %sms %smks %sns"%(secs,msecs,mksecs,nsecs)

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
	t3 = rospy.Time.now()

	dp = np.array([0., 0., 0.])
	dq = np.array([0., 0., 0., 1.])
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
			p = p+dp
			q = tr.quaternion_multiply(q,dq)
			SE3 = getSE3_matrix(p,q)
			
			
			camera_pose_cell = position_to_cell(p, scale)
			print(camera_pose_cell)
			
			#cloud_out = transform_cloud(SE3, trans.header.frame_id, cloud)
			#pts = cloud_out.values
			t1 = rospy.Time.now()
			print("pub markers:  ", time_to_s_ms_ns(t1 - t3))
			#print(cloud_out.values.shape)
			#occupied_cells = np.transpose(positions_to_cell(pts[0:-1,:],scale))
			(best_camera_pose_meters,
			best_camera_quaternion,
			best_camera_cells,
			best_cloud_cells,
			delta_pose,
			delta_quater) = find_best_pose(m, cloud, p, q, scale, odom_frame)
			cloud_out = transform_cloud(getSE3_matrix(best_camera_pose_meters, best_camera_quaternion),
			                            odom_frame, cloud)
			#print(delta_pose)
			#print(delta_quater)
			dp += delta_pose
			dq = tr.quaternion_multiply(dq,delta_quater)
			line = bresenhamline(best_cloud_cells, best_camera_cells,-1)
			t2 = rospy.Time.now()
			print("scan matcher: ", time_to_s_ms_ns(t2 - t1))
			print(type(line[0][0]).__name__)
			m.update_cells(line,np.array([0.05, 0.9, 0.05]))
			m.update_cells(best_cloud_cells,np.array([0.9, 0.05, 0.05]))
			t3 = rospy.Time.now()
			print("update cells: ", time_to_s_ms_ns(t3 - t2))
				

			pub1.publish(cloud_to_marker(cloud_out,1,0,0))
			pub2.publish(m.to_marker())
			#pub2.publish(cloud_to_marker(cloud,0,1,0))
			#r.sleep()



	rospy.loginfo("after_for")
















