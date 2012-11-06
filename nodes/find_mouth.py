#!/usr/bin/env python
import roslib; roslib.load_manifest('mouth_click')
import cv2
import image_geometry
import cv_bridge
import numpy as np
import rospy
from people_msgs.msg import PositionMeasurement
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty
import tf

bridge = cv_bridge.CvBridge()
model = image_geometry.PinholeCameraModel()
mouth_offset = None
mouth_center = None
depth_img = None
color_img = None
tfl = None

def info_cb(msg):
	model.fromCameraInfo(msg)

def depth_im_cb(msg):
	global depth_img, color_img
	depth_img = np.asarray(bridge.imgmsg_to_cv(msg))
	color_img = cv2.cvtColor(depth_img, cv2.cv.CV_GRAY2RGB)

def people_cb(meas):
	global mouth_center
	ps = PointStamped()
	ps.point = meas.pos
	ps.header = meas.header
	ps.header.stamp = rospy.Time(0)
	point = tfl.transformPoint(model.tf_frame, ps)
	mouth_center = model.project3dToPixel((point.point.x, point.point.y+mouth_offset, point.point.z))

if __name__ == '__main__':
	rospy.init_node('find_mouth')

	do_display = rospy.get_param('do_display', default=True)
	mouth_offset = rospy.get_param('mouth_offset', default=0.05)

	if do_display:
		cv2.namedWindow('Mouth')

	tfl = tf.TransformListener()
	camera = rospy.resolve_name('camera')
	rospy.Subscriber(camera + '/depth_registered/image_rect'     , Image, depth_im_cb)
	rospy.Subscriber(camera + '/depth_registered/camera_info'    , CameraInfo, info_cb)
	rospy.Subscriber('/face_detector/people_tracker_measurements', PositionMeasurement, people_cb)

	rospy.Publisher('click', Empty)

	rospy.loginfo('Waiting for face to be ready')
	while (mouth_center is None or depth_img is None) and not rospy.is_shutdown():
		rospy.sleep(0.1)
	rospy.loginfo('Ready')

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if do_display:
			cv2.circle(color_img, (int(mouth_center[0]), int(mouth_center[1])), 5, (0,255,0), 5)
			cv2.imshow('Mouth', color_img)
			cv2.waitKey(10)

		print depth_img[mouth_center[1], mouth_center[0]]
		rate.sleep()