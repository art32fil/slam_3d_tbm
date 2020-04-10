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
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
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
from bresenham import _bresenhamlines
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
	a = np.array(tr.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]))
	return p,a

def getSE3_matrix(pose,angels):
	# pose = np.array([x, y, z])
	# angels = np.array([ax, ay, az])
	g = tr.euler_matrix(angels[0], angels[1], angels[2])
	g[0:3, -1] = pose
	return g
	

def transformStamped_to_SE3(transform):
	if not isinstance(transform, TransformStamped):
		rospy.logwarn("try to convert tansform of incorrect type.\nThe type is %s\nThe possible type is %s",
		              type(transform).__name__, "TransformStamped")
	p,a = get_pose_angle(transform)
	g = getSE3_matrix(p,a)
	return g

def transform_cloud(SE3, new_frame, cloud):
	cloud_out = Cloud()
	cloud_out.name = new_frame
	cloud_out.values = np.matmul(SE3, cloud.values)
	return cloud_out

def cost_of_scan(grid_map, scan, robot_pose):
	occupied_points = scan + np.array([grid_map.Zx, grid_map.Zy, grid_map.Zz])
	occ_cells = grid_map.map[occupied_points[:,0],occupied_points[:,1],occupied_points[:,2]]
	value = np.sum(cell_to_prob(merge_cells(occ_cells, np.array([0.9,0.5,0.5]))))

	return value#accumulator
		
def find_best_pose(grid_map, scan_meters, robot_pose_meters, robot_angles, scale, world_frame):
	
	final_pose = robot_pose_meters
	final_angles = robot_angles
	SE3 = getSE3_matrix(final_pose, final_angles)
	cloud_out = transform_cloud(SE3, world_frame, scan_meters)
	cloud_cells = positions_to_cell(np.transpose(cloud_out.values[0:-1]), scale)
	robot_cells = position_to_cell(robot_pose_meters, scale)
	max_cost = cost_of_scan(grid_map, cloud_cells, robot_cells)
	final_robot_cells = robot_cells
	final_cloud_cells = cloud_cells
	final_pose_delta = np.array([0.0, 0.0, 0.0])
	final_angles_delta = np.array([0.0, 0.0, 0.0])
	for i in range(1000):
		delta_pose = 0.5*scale*np.random.randn(3)
		delta_angles = 0.01*np.random.randn(3)
		if scan_matcher_is_used == False:
			delta_pose = np.array([0,0,0])
			delta_angles = np.array([0,0,0])
		new_pose = final_pose + delta_pose
		new_angles = final_angles + delta_angles
		new_SE3 = getSE3_matrix(new_pose, new_angles)
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
		#print("min_cost: ", min_cost, ", cost: ", cost)
		#print("cost_of_scan executes:    ", rospy.Time.now() - t)
		#print("after cost_of_scan")
		if cost > max_cost*1.05:
			max_cost = cost
			final_pose = new_pose
			final_angles = new_angles
			final_robot_cells = robot_cells
			final_cloud_cells = cloud_cells
			final_pose_delta = delta_pose
			final_angles_delta = delta_angles
	return final_pose, final_angles, final_robot_cells, final_cloud_cells, final_pose_delta, final_angles_delta

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
	np.random.seed(int(rospy.get_param('~seed')))
	rospy.loginfo("begin")
	pose_out_file = open(str(rospy.get_param('~out/file_poses')), 'w')

	bag_name = str(rospy.get_param('~bag/file_path'))
	bag = rosbag.Bag(bag_name, 'r')
	topic_pcl2_name = str(rospy.get_param('~bag/topic_pcl'))

	world_frame = str(rospy.get_param('~tf/world_frame'))
	odom_frame = str(rospy.get_param('~tf/odom_frame'))
	camera_frame = str(rospy.get_param('~tf/pcl_frame'))

	scan_matcher_is_used = bool(rospy.get_param('~sm/allow_scan_matcher'))
	
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
	dp_abs = np.array([0., 0., 0.])
	da = np.array([0., 0., 0.])
	pose = np.array([0., 0., 0.])
	angle = np.array([0., 0., 0.])
	for topic, msg, time in bag.read_messages(topics = [topic_pcl2_name, "/tf", "/tf_static"]):
		if rospy.is_shutdown():
			break
		if topic == "/tf" or topic == "/tf_static":
			transorms = msg
			for msg_tf in transorms.transforms:
        			tf_bc.sendTransform(msg_tf)
			msg_tf = TransformStamped()
			msg_tf.header.stamp = time
			msg_tf.header.frame_id = world_frame
			msg_tf.child_frame_id = odom_frame
			msg_tf.transform.translation = Vector3(dp[0],dp[1],dp[2])
			q = tr.quaternion_from_euler(da[0],da[1],da[2])
			msg_tf.transform.rotation = Quaternion(q[0],q[1],q[2],q[3])
			tf_bc.sendTransform(msg_tf)
		elif topic == topic_pcl2_name:
			cloud = Cloud(msg.name, np.reshape(np.array(msg.values, dtype=np.float32),(4,-1),'F'))
			#pts = pcl2_handler.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
			pts = cloud.values
			#print(pts.shape)
			trans = get_tf_transform(in_which_frame = world_frame, what_frame = camera_frame,
			                      tf_buffer = tf_buffer)
			if (trans == None):
				continue
			p,a = get_pose_angle(trans)
			print("p: ", p, " a: ", a)
			pose = p
			angle = a
			#SE3 = getSE3_matrix(p,a)
			
			
			camera_pose_cell = position_to_cell(p, scale)
			#print(camera_pose_cell)
			
			#cloud_out = transform_cloud(SE3, trans.header.frame_id, cloud)
			#pts = cloud_out.values
			t1 = rospy.Time.now()
			print("pub markers:  ", time_to_s_ms_ns(t1 - t3))
			#print(cloud_out.values.shape)
			#occupied_cells = np.transpose(positions_to_cell(pts[0:-1,:],scale))
			(best_camera_pose_meters,
			best_camera_angles,
			best_camera_cells,
			best_cloud_cells,
			delta_pose,
			delta_angle) = find_best_pose(m, cloud, pose, angle, scale, world_frame)
			
			pose_quaternion = tr.quaternion_from_euler(best_camera_angles[0], best_camera_angles[1], best_camera_angles[2])
			pose_out_file.write(str(time.to_sec())+" "+
			                   str(best_camera_pose_meters[0])+" "+ str(best_camera_pose_meters[1]) + " " + str(best_camera_pose_meters[2])+ " "+
			                   str(pose_quaternion[0])+" "+str(pose_quaternion[1])+" "+str(pose_quaternion[2])+" "+str(pose_quaternion[3])+"\n")

			cloud_out = transform_cloud(getSE3_matrix(best_camera_pose_meters, best_camera_angles),
			                            world_frame, cloud)
			print("dp: ", positions_to_cell(delta_pose,scale), " da: ", delta_angle)
			trans = get_tf_transform(in_which_frame = world_frame, what_frame = odom_frame,
			                      tf_buffer = tf_buffer)
			p,a = get_pose_angle(trans)
			dp = p + delta_pose
			da = a + delta_angle
			print("dp_: ", positions_to_cell(dp,scale), " da_ ", da)
			lines = _bresenhamlines(best_cloud_cells, best_camera_cells,-1)
			t2 = rospy.Time.now()
			print("scan matcher: ", time_to_s_ms_ns(t2 - t1))
			#print(type(line[0][0]).__name__)
			#m.update_cells(line,np.array([0.05, 0.9, 0.05]))
			#m.update_cells(best_cloud_cells,np.array([0.9, 0.05, 0.05]))
			for line in lines:
				l = -0.5
				exp = 0.95*np.exp(l*np.arange(0,len(line)))
				cell_values = np.transpose(np.array([exp, 0.95 - exp, 0.05*np.ones(len(line))]))
				m.update_cells(line,cell_values)
			t3 = rospy.Time.now()
			print("update cells: ", time_to_s_ms_ns(t3 - t2))
				

			#pub1.publish(cloud_to_marker(cloud_out,1,0,0))
			#pub2.publish(m.to_marker())
			#pub2.publish(cloud_to_marker(cloud,0,1,0))
			#r.sleep()
			#input("Press Enter to continue...")


	pose_out_file.close()
	rospy.loginfo("after_for")
















