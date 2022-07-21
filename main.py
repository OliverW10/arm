import math
from camera_apriltags import Vision
import cv2
import serial
import glob
import time
import ik

def sendAngles(ser, yaw, pitch, wrist):
    yaw = yaw + ik.offset_1
    pitch = pitch + ik.offset_2
    wrist = wrist + ik.offset_3
    string = ",".join(str(round(x)) for x in [yaw, pitch, wrist])+";"
    ser.write(bytes(string, "utf-8"))

cap = cv2.VideoCapture(0)

serial_devices = glob.glob("/dev/ttyACM*")
if len(serial_devices) == 0:
    print("arduino not found")
ser = serial.Serial(serial_devices[0])

vision = Vision(True, True)

while True:
    start = time.time()
    ret, frame = cap.read()
    # print("capture time", time.time()-start)
    if not ret or frame is None:
        print("no camera frame")

    start = time.time()
    offset = vision.run(frame)
    # print("process time", time.time()-start)

    if not offset is None:
        joint_angles = ik.getAngles(offset)
        print(joint_angles)
        start = time.time()
        sendAngles(ser, *joint_angles)
    # print("send time", time.time()-start)
    
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
