import math
from camera_apriltags import Vision
import cv2
import serial
import glob
import time
import ik
from util import clamp, rate_limit


def sendAngles(ser, yaw, pitch, wrist):
    yaw = clamp(yaw + ik.offset_1, 0, 180)
    pitch = clamp(pitch + ik.offset_2, 0, 180)
    wrist = clamp(wrist + ik.offset_3, 0, 180)
    string = ",".join(str(round(x)) for x in [yaw, pitch, wrist])+";"
    ser.write(bytes(string, "utf-8"))

cap1 = cv2.VideoCapture(0)

serial_devices = glob.glob("/dev/ttyACM*")
print(serial_devices)
if len(serial_devices) == 0:
    print("arduino not found")
ser = serial.Serial(serial_devices[0])

vision1 = Vision(cap1, "cam1", True)
joint_angles = [90, 90, 90]
program_start = time.time()
while True:
    # position of target relative to arm
    offset = [0, 0, 0]
    got_new_data = 0
    for vision in [vision1]:
        new_offset = vision.run()
        if new_offset is not None:
            got_new_data += 1
            offset = [a+b for a,b in zip(offset, new_offset)]


    if got_new_data:
        # print(offset)
        offset = [x/got_new_data for x in offset]
        new_joint_angles = ik.getAngles(offset, 0.01)

        start = time.time()
        # TODO: filtering
        joint_angles = new_joint_angles
        sendAngles(ser, *joint_angles)
        send_time = time.time()-start
        if send_time > 0.5:
            ser.close()
            time.sleep(0.2)
            ser.open()

    if cv2.waitKey(1) == ord("q"):
        break

cap1.release()
cv2.destroyAllWindows()
ser.close()
