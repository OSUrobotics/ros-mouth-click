#!/usr/bin/env rosh
import roslib; roslib.load_manifest('mouth_click')
import cv2
import image_geometry
import cv_bridge
import numpy as np
bridge = cv_bridge.CvBridge()

cv2.namedWindow('win')
model = image_geometry.PinholeCameraModel()

while ok():
	info_msg = topics.camera.rgb.camera_info[0]
	model.fromCameraInfo(info_msg)
	meas = topics.face_detector.people_tracker_measurements[0]

	mouth_center = model.project3dToPixel((meas.pos.x, meas.pos.y+0.07, meas.pos.z))

	img_msg = topics.camera.rgb.image_rect[0]
	img = np.asarray(bridge.imgmsg_to_cv(img_msg))
	img = cv2.cvtColor(img, cv2.cv.CV_GRAY2RGB)
	cv2.circle(img, (int(mouth_center[0]), int(mouth_center[1])), 5, (0,255,0), 5)

	cv2.imshow('win', img)
	cv2.waitKey(5)