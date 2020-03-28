#! /usr/bin/env python3

import rospy
import rosbag
import numpy as np
from ros_numpy import point_cloud2
from sensor_msgs.msg import ChannelFloat32
import math
from math import isnan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from tf.msg import tfMessage

def world_to_cell(point, scale):
	i = int(math.floor(point[0]/scale))
	j = int(math.floor(point[1]/scale))
	k = int(math.floor(point[2]/scale))
	return (i,j,k)

if __name__ == '__main__':
	rospy.init_node("tester")
	#pub = rospy.Publisher("m_topic",Marker,queue_size=10)
	#pub_tf = rospy.Publisher("/tf",tfMessage,queue_size=100)
	bag_in = rosbag.Bag("/home/west/Downloads/rgbd_dataset_freiburg1_room-2hz-with-pointclouds.bag", 'r')
	bag_out = rosbag.Bag("/home/west/Downloads/rgbd_dataset_freiburg1_room-2hz-with-pointclouds-1700points_per_scan-4D.bag", 'w')
	for topic, msg, time in bag_in.read_messages():
		if rospy.is_shutdown():
			break
		'''if topic == "/tf":
			for tr in msg.transforms:
				tr.header.stamp = rospy.Time.now()
			pub_tf.publish(msg)'''
			
		if topic == "/camera/depth/points":
			point_cloud = msg
			pts = point_cloud2.pointcloud2_to_array(point_cloud)
			d = {}
			for raw in pts:
				for pt in raw:
					pt = np.array([pt[0], pt[1], pt[2], 1])
					if isnan(pt[0])  or isnan(pt[1]) or isnan(pt[2]):
						continue
					cell = world_to_cell(pt, 0.05)
					if cell in d:
						d[cell] = np.append(d[cell],pt)
					else:
						d[cell] = np.array([pt])
			float_array = np.array([])

			'''marker_msg = Marker()
			marker_msg.header.stamp = rospy.Time.now()
			marker_msg.header.frame_id = point_cloud.header.frame_id
			marker_msg.ns = 'a'
			marker_msg.id = 0
			marker_msg.type = Marker.POINTS
			marker_msg.action = Marker.ADD
			marker_msg.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
			marker_msg.scale = Vector3(0.01,0.01,0.01)
			marker_msg.color = ColorRGBA(1,0,0,0.5)
			marker_msg.lifetime = rospy.Duration(0)'''
			points = []
			
			for k,v in d.items():
				avg_point = np.average(np.reshape(v,(-1,4)), axis=0)
				float_array = np.append(float_array, avg_point)
				points.append(Point(avg_point[0], avg_point[1], avg_point[2]))
			#marker_msg.points = points
			print(len(points))
			
			#pub.publish(marker_msg)
			
			msg_sparse_point_cloud = ChannelFloat32()
			msg_sparse_point_cloud.name = point_cloud.header.frame_id
			msg_sparse_point_cloud.values = [np.float32(x) for x in float_array]
			msg = msg_sparse_point_cloud
				
		bag_out.write(topic, msg, time)
	bag_out.close()

'''import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float32MultiArray

import numpy

def callback(data):
    #print(rospy.get_name(), "I heard %s"%str(data.data))
    a = numpy.array(data.data,dtype=numpy.float32)
    a1 = numpy.reshape(a,(2,-1))
    print(a1)

def talker():
    pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("floats", numpy_msg(Floats), callback)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        a = numpy.array([[1.0, 2.1, 3.2, 4.3, 5.4, 6.5], [2.0, 3.1, 3.2, 4.3, 5.4, 6.5]],dtype=numpy.float32)
        a1 = numpy.reshape(a,len(a)*len(a[0]))
        pub.publish(a1)
        r.sleep()

if __name__ == '__main__':
    talker()'''
	
