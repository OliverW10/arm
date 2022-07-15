import math
import time
import cv2
from dt_apriltags import Detector, Detection
from typing import List
import numpy as np

from util import project


# camera matrix explenation
# https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html
# better: https://www.cs.cmu.edu/~16385/s17/Slides/11.1_Camera_matrix.pdf
# focal distance
# https://www.chiefdelphi.com/t/focal-length-of-microsoft-lifecam/157480
scale_ratio = 2
fx, fy = 333, 333
fx, fy = fx/scale_ratio, fy/scale_ratio
cxr, cyr = 0.5, 0.5

tag_size = 0.05
arm_tag_id = 1
goal_tag_id = 9

def good_tag(tag: Detection) -> bool:
    return tag.hamming <= 1 and tag.decision_margin > 1.5

at_detector = Detector(searchpath=['apriltags'],
                       families='tag16h5',
                       nthreads=4,
                       quad_decimate=2.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

vid = cv2.VideoCapture(0)
first = True
grey = None
frame_counter = 0

debug = True

rel_pos_arm = np.array([0, 0, 0])
rel_pos_world = np.array([0, 0, 0])

while True:
    start_time = time.perf_counter()
    ret, frame = vid.read()
    frame = cv2.resize(frame, None, fx=1/scale_ratio, fy=1/scale_ratio)
    
    if not ret:
        print("no image, try different camera")
        break

    # if its the first frame allocate images
    if first:
        first = False
        print("frame shape", frame.shape[:2])
        grey = np.zeros(frame.shape[:2], dtype=np.uint8)

    # apriltags work on greyscale images
    cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, grey, 0)
    tags: List[Detection] = at_detector.detect(grey, estimate_tag_pose=True, camera_params=[fx, fy, cxr*grey.shape[1], cyr*grey.shape[0]], tag_size=tag_size)
    # # remove bad tags
    tags = list(filter(good_tag, tags))
    tag_nums = [t.tag_id for t in tags]

    if arm_tag_id in tag_nums and goal_tag_id in tag_nums:
        has_targets = True
        arm_tag: Detection = list(filter(lambda x:x.tag_id==arm_tag_id, tags))[0]
        arm_pos = np.reshape(arm_tag.pose_t, (3,))
        # will replace with average of poses of all goal tags
        goal_tag: Detection = list(filter(lambda x:x.tag_id==goal_tag_id, tags))[0]
        goal_pos = np.reshape(goal_tag.pose_t, (3,))
        # position of goal from arm in world coordinate system
        rel_pos_world = np.array([goal_pos[i]-arm_pos[i] for i in range(3)])*0.5
        # get arm to goal in arm coordinate space
        # rotate by arm rotation
        rel_pos_arm = rel_pos_world@arm_tag.pose_R
        # print("arm to target:", [round(x, 3) for x in rel_pos_arm])
    else:
        has_targets = False

    if debug:
        for tag in tags:
            # draw rect around corners
            corners = np.asarray(tag.corners, dtype=int)
            cv2.polylines(frame, [corners], True, (0, 0, 255), 3)
            # draw tag num
            cv2.putText(frame, "#"+str(tag.tag_id), (round(corners[0][0]), round(corners[0][1])), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
            
            # draw axies
            w2 = round(frame.shape[1]/2)
            h2 = round(frame.shape[0]/2)
            tag_x = tag.pose_t[0][0]
            tag_y = tag.pose_t[1][0]
            tag_z = tag.pose_t[2][0]
            x_axis = tag.pose_R@np.array([tag_size, 0, 0])
            y_axis = tag.pose_R@np.array([0, tag_size, 0])
            z_axis = tag.pose_R@np.array([0, 0, -tag_size])
            for _axis in (x_axis, y_axis, z_axis):
                axis = np.reshape(_axis, (3, ))
                x1, y1 = project(tag_x, tag_y, tag_z, fx, fy)
                axis_pos3 = axis+np.reshape(tag.pose_t, (3, ))
                x2, y2 = project(axis_pos3[0], axis_pos3[1], axis_pos3[2], fx, fy)
                cv2.line(frame, (round(x1)+w2, round(y1)+h2), (round(x2)+w2, round(y2)+h2), (0, 255, 0))

        if has_targets:
            w2 = round(frame.shape[1]/2)
            h2 = round(frame.shape[0]/2)
            arm_tag: Detection = list(filter(lambda x:x.tag_id==arm_tag_id, tags))[0]
            arm_tag_t = np.reshape(arm_tag.pose_t, (3,))
            x1, y1 = project(arm_tag_t[0], arm_tag_t[1], arm_tag_t[2], fx, fy)
            end_pos = arm_tag_t + rel_pos_world
            x2, y2 = project(end_pos[0], end_pos[1], end_pos[2], fx, fy)
            cv2.line(frame, (round(x1)+w2, round(y1)+h2), (round(x2)+w2, round(y2)+h2), (255, 0, 0), 3)

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord("q"):
        break

    frame_counter += 1
    if frame_counter > 30:
        frame_counter = 0
        print("fps:", round(1/(time.perf_counter()-start_time), 1))

vid.release()
cv2.destroyAllWindows()