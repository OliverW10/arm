import math
from camera_apriltags import Vision
import cv2
import serial
import glob
import time

def sendAngles(ser, yaw, pitch, wrist):
    string = ",".join(str(round(x)) for x in [yaw, pitch, wrist])+";"
    print(string)
    ser.write(bytes(string, "utf-8"))

cap = cv2.VideoCapture(0)

serial_devices = glob.glob("/dev/ttyACM*")
if len(serial_devices) == 0:
    print("plug in the arduino you fuckhead")
ser = serial.Serial(serial_devices[0])

vision = Vision(True, True)

while True:
    start = time.time()
    ret, frame = cap.read()
    # print("read time", time.time()-start)
    if not ret or frame is None:
        print("wrong camera id fuckhead")

    start = time.time()
    offset = vision.run(frame)
    # print("process time", time.time()-start)

    if not offset is None:
        yaw = math.degrees(math.atan2(offset[1], offset[0])+math.pi)
        if yaw > 180:
            # goto nearest edge
            if yaw < 270:
                yaw = 180
            else:
                yaw = 0

        start = time.time()
        sendAngles(ser, 180-yaw, 0, 0)
        print("send time", time.time()-start)
    
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
