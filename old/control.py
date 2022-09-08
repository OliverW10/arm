import math
import serial
import glob
import time
from util import clamp
import ik
import pygame

def sendAngles(ser, yaw, pitch, wrist):
    yaw = clamp(yaw + ik.offset_1, 0, 180)
    pitch = clamp(pitch + ik.offset_2, 0, 180)
    wrist = clamp(wrist + ik.offset_3, 0, 180)
    string = ",".join(str(round(x)) for x in [yaw, pitch, wrist])+";"
    ser.write(bytes(string, "utf-8"))

pygame.init()
screen = pygame.display.set_mode((800,600))
pygame.display.set_caption('arm control')

serial_devices = glob.glob("/dev/ttyACM*")
print(serial_devices)
if len(serial_devices) == 0:
    print("arduino not found")
ser = serial.Serial(serial_devices[0])

program_start = time.time()
running = True
scale = 0.4
height = 0
height_vel = 0
height_max_speed = 0.1

while running:
    screen.fill((0, 0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                height_vel -= height_max_speed
            if event.key == pygame.K_DOWN:
                height_vel += height_max_speed
        if event.type == pygame.KEYUP:
            if event.key == pygame.K_UP:
                height_vel += height_max_speed
            if event.key == pygame.K_DOWN:
                height_vel -= height_max_speed

    height += height_vel*0.05
    mouse_x, mouse_y = pygame.mouse.get_pos()
    x, y = mouse_x/400-1, mouse_y/300-1
    goal = [scale*y, scale*x, height]
    # goal = [ik.l1+ik.l2+ik.l3, 0, 0]
    angles = ik.getAngles(goal)
    # print("angles", [round(x, 2) for x in angles])
    sendAngles(ser, *angles)
    # sendAngles(ser, 90, 90, 0)
    
    pygame.draw.circle(screen, (255, 0, 0), (400, 300), 10)
    pygame.draw.circle(screen, (255, 0, 0), (mouse_x, mouse_y), 10)
    pygame.display.update()
    time.sleep(0.05)

ser.close()
pygame.quit()
quit()