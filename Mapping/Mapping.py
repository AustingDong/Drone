from djitellopy import tello
import KeyPressModule as kp
import cv2
import numpy as np
from time import sleep
import time
import math

kp.init()
me = tello.Tello()
me.connect()

print(me.get_battery())





###############PARAS###############
forward_speed = 117/10
angular_speed = 360/10
interval = 0.25

distance_interval = forward_speed * interval
angular_interval = angular_speed * interval

x,y = 500,500
ang = 0
yaw = 0

###################################

def drawPoints(locate_map, points):
    cv2.circle(locate_map, (points[0], points[1]), 5, (255, 255, 255), cv2.FILLED)  #BGR

def directFacing(locate_map, points, facing_points):
    cv2.line(locate_map, (points[0], points[1]),(facing_points[0], facing_points[1]), (0, 0, 255), thickness=1, shift=0)

def getKeyboardInput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 50
    d = 0
    global x, y, yaw, ang, facing_ang
    if kp.getKey("a"): 
        lr = -speed
        d = distance_interval
        ang = -180

    elif kp.getKey("d"): 
        lr = speed
        d = -distance_interval
        ang = 180

    if kp.getKey("UP"): ud = speed
    elif kp.getKey("DOWN"): ud = -speed

    if kp.getKey("w"): 
        fb = speed
        d = distance_interval
        ang = 270

    elif kp.getKey("s"): 
        fb = -speed
        d = -distance_interval
        ang = -90

    if kp.getKey("LEFT"):
        yv = -speed
        yaw -= angular_interval

    elif kp.getKey("RIGHT"):
        yv = speed
        yaw += angular_interval

    if kp.getKey("g"): me.land()
    elif kp.getKey("f"): me.takeoff()

    if kp.getKey("q"):
        cv2.imwrite(f'./Images/{time.time()}.jpg', img)
        print("successfully took the photo")
        sleep(0.3)


    ang += yaw
    
    x += int(d*math.cos(math.radians(ang)))
    y += int(d*math.sin(math.radians(ang)))

    facing_x = x + int(30 * math.cos(math.radians(yaw)))
    facing_y = y + int(30 * math.sin(math.radians(yaw)))
    return [lr, fb, ud, yv, x, y, facing_x, facing_y]

while True:
    vals = getKeyboardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    
    locate_map = np.zeros((1000, 1000, 3), np.uint8)
    points = (vals[4], vals[5])
    facing_points = (vals[6], vals[7])
    drawPoints(locate_map, points)
    directFacing(locate_map, points, facing_points)
    cv2.imshow("locate", locate_map)
    
    sleep(0.01)