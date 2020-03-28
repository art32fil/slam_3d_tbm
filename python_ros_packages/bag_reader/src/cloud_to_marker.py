from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import rospy

def cloud_to_marker(cloud,r,g,b):
	# cloud has following atribures:
	#	cloud.name - coresponded tf frame
	#	cloud.values - 2D array of points (each col is a point)
	marker_msg = Marker()
	marker_msg.header.stamp = rospy.Time.now()
	marker_msg.header.frame_id = cloud.name
	marker_msg.ns = 'a'
	marker_msg.id = 0
	marker_msg.type = Marker.POINTS
	marker_msg.action = Marker.ADD
	marker_msg.pose = Pose(Point(0,0,0), Quaternion(0,0,0,1))
	marker_msg.scale = Vector3(0.03,0.03,0.01)
	marker_msg.color = ColorRGBA(r,g,b,0.5)
	marker_msg.lifetime = rospy.Duration(0)
	marker_msg.points = []

	for i in range(len(cloud.values[0])):
		pt = Point(cloud.values[0][i], cloud.values[1][i], cloud.values[2][i])
		marker_msg.points.append(pt)
	return marker_msg
		
