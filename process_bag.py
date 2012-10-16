#!/usr/bin/env python
roslib.load_manifest('cv_bridge')
import cv2, numpy as np
import cv_bridge

UP = 0
DN = 1

LEFT    = 0
MIDDLE  = 1
RIGHT   = 2
NOCLICK = 3

MOUSE_STATUS = {
	0 : (NOCLICK, UP),
	1 : (LEFT,    DN),
	2 : (RIGHT,   DN),
	3 : (MIDDLE,  DN),
	4 : (LEFT,    UP),
	5 : (RIGHT,   UP),
	6 : (MIDDLE,  UP)
}

BACK    = -1
PAUSE   =  0
FORWARD =  1
STEP    =  2

state = FORWARD

def mouse_cb(click, x, y, a1, a2):
	global state
	button, down = MOUSE_STATUS[click]		
	if button == MIDDLE and not down:
		# state = not state
		print 'Frame %s: %s' % (idx, img[y,x])
	if button == LEFT and not down:
		state = BACK
	if button == RIGHT and not down:
		state = STEP

topic = '/face_cam/depth_registered/image_rect'
bridge = cv_bridge.CvBridge()
cv2.namedWindow(topic)
cv2.setMouseCallback(topic, mouse_cb)

img = None
images = []
idx = 0

def trackbar_cb(pos):
	global idx
	idx = pos

with Bag('mouth_with_face.bag') as bag:
	for topic, msg, t in bag.read_messages(topics=[topic, '/face_detector/people_tracker_measurements']):
		if hasattr(msg, 'pos'):
			pass
		else:
			img = np.asarray(bridge.imgmsg_to_cv(msg))
			images.append(img)
		
cv2.createTrackbar('Frame', topic, 0, len(images)-1, trackbar_cb)
		
while True:
	if state == FORWARD:
		idx = min(idx+1,len(images)-1)
	if state == BACK:
		idx = max(idx-1,0)
		state = PAUSE
	if state == STEP:
		idx = min(idx+1,len(images)-1)
		state = PAUSE
	cv2.setTrackbarPos('Frame', topic, idx)
	cv2.imshow(topic, 255*np.uint8(np.isnan(images[idx])))
	key = cv2.waitKey(20)
	if key == 27:
		break
	if key == 32:
		state = not state
