import math
import time
import cv2
from dt_apriltags import Detector, Detection
from typing import List, Optional, Tuple, Union
import numpy as np
from util import project


# camera matrix explenation
# https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html
# better: https://www.cs.cmu.edu/~16385/s17/Slides/11.1_Camera_matrix.pdf
# focal distance
# https://www.chiefdelphi.com/t/focal-length-of-microsoft-lifecam/157480
scale_ratio = 1
fx, fy = 333, 333
fx, fy = fx/scale_ratio, fy/scale_ratio
cxr, cyr = 0.5, 0.5

tag_size = 0.038

servo_height = -0.035
arm_tag_ids = {
    6:[0, 0.055, servo_height],
    7:[0.055, 0, servo_height],
    8: [0, -0.05, servo_height],
}
goal_tag_ids = [0, 1, 2, 3, 4, 5]
cube_size = 0.05

# target/cube: where the focused object is
# goal: where we want the end effector to be

def good_tag(tag: Detection) -> bool:
    return tag.hamming <= 1 and tag.decision_margin > 1.5

class Vision:
    def __init__(self, display=True, debug=True):
        self.at_detector = Detector(searchpath=['apriltags'],
                            families='tag16h5',
                            nthreads=4,
                            quad_decimate=2.0,
                            quad_sigma=0.0,
                            refine_edges=1,
                            decode_sharpening=0.25,
                            debug=0)
        self.first = True
        self.grey = None
        self.frame_counter = 0
        self.debug = debug
        self.display = display


        self.target_pos = np.array([0, 0, 0], dtype=np.float64)
        self.cube_pos = np.array([0, 0, 0], dtype=np.float64)
        self.arm_pos = np.array([0, 0, 0], dtype=np.float64)

    def run(self, frame: np.ndarray) -> Optional[Tuple[float, float, float]]:
        start_time = time.perf_counter()
        frame = cv2.resize(frame, None, fx=1/scale_ratio, fy=1/scale_ratio)
        
        # if its the first frame allocate images
        if self.first:
            self.first = False
            print("frame shape", frame.shape[:2])
            self.grey = np.zeros(frame.shape[:2], dtype=np.uint8)

        # apriltags work on greyscale images
        cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY, self.grey, 0)
        tags: List[Detection] = self.at_detector.detect(self.grey, estimate_tag_pose=True, camera_params=[fx, fy, cxr*self.grey.shape[1], cyr*self.grey.shape[0]], tag_size=tag_size)
        # # remove bad tags
        tags = list(filter(good_tag, tags))

        # average offset positions for all tags on cube
        target_tags = [t for t in tags if t.tag_id in goal_tag_ids]
        if len(target_tags):
            self.target_pos = np.array([0, 0, 0], dtype=np.float64)
            for tag in target_tags:
                # find the center of the cube from the tag pose
                cube_pos = np.reshape(tag.pose_t, (3,)) + tag.pose_R@[0, 0, cube_size/2]
                self.target_pos += cube_pos
            # average cube centers from all tags
            self.target_pos /= len(target_tags)

        # average offset positions of all arm tags
        arm_tags = [t for t in tags if t.tag_id in arm_tag_ids.keys()]
        if len(arm_tags):
            self.arm_pos = np.array([0, 0, 0], dtype=np.float64)
            for tag in arm_tags:
                arm_pos = np.reshape(tag.pose_t, (3,)) + (tag.pose_R@arm_tag_ids[tag.tag_id])
                self.arm_pos += arm_pos
            self.arm_pos /= len(arm_tags)

        if len(arm_tags) and len(target_tags):
            has_targets = True
            # position of goal from arm in world coordinate system
            rel_pos_world = self.target_pos - self.arm_pos
            # get arm to goal in arm coordinate space
            # rotate by arm rotation
            rel_pos_arm = rel_pos_world@arm_tags[0].pose_R
        else:
            has_targets = False

        # draw debug visuals
        if self.debug:
            # half width, half height
            w2 = round(frame.shape[1]/2)
            h2 = round(frame.shape[0]/2)
            for tag in tags:
                # draw rect around corners
                corners = np.asarray(tag.corners, dtype=int)
                cv2.polylines(frame, [corners], True, (0, 0, 255), 3)
                # draw tag num
                cv2.putText(frame, "#"+str(tag.tag_id), (round(corners[0][0]), round(corners[0][1])), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
                
                if tag.tag_id in goal_tag_ids:
                    offset = [0, 0, cube_size/2]
                elif tag.tag_id in arm_tag_ids.keys():
                    offset = arm_tag_ids[tag.tag_id]
                else:
                    continue

                # reshapes the pos from [[x], [y], [z]] to [x, y, z]
                tag_pos = np.reshape(tag.pose_t, (3, ))
                # rotated_offset = tag.pose_R@np.array(offset)
                rotated_offset = tag.pose_R@np.array(offset)
                actual_pos = rotated_offset+tag_pos
                # gets screen position of tag center
                x1, y1 = project(tag_pos[0], tag_pos[1], tag_pos[2], fx, fy)
                # gets screen position of object center
                x2, y2 = project(actual_pos[0], actual_pos[1], actual_pos[2], fx, fy)
                cv2.line(frame, (round(x1)+w2, round(y1)+h2), (round(x2)+w2, round(y2)+h2), (0, 255, 0), 3)

            if has_targets:
                # show line from arm to cube
                x1, y1 = project(self.arm_pos[0], self.arm_pos[1], self.arm_pos[2], fx, fy)
                x2, y2 = project(self.target_pos[0], self.target_pos[1], self.target_pos[2], fx, fy)
                cv2.line(frame, (round(x1)+w2, round(y1)+h2), (round(x2)+w2, round(y2)+h2), (255, 0, 0), 3)

        if self.display:
            cv2.imshow('frame', frame)

        self.frame_counter += 1
        if self.frame_counter > 30 and self.debug:
            self.frame_counter = 0
            # print("fps:", round(1/(time.perf_counter()-start_time), 1))
        
        if has_targets:
            return rel_pos_arm

    def __del__(self):
        cv2.destroyAllWindows()
