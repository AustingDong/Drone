
import KeyPressModule as kp
import cv2
import numpy as np
from time import sleep
import time
import math

kp.init()





############### PARAS ###############
forward_speed = 117/10
angular_speed = 360/10
interval = 0.25

distance_interval = forward_speed * interval
angular_interval = angular_speed * interval

x,y = 500,500
ang = 0
yaw = 0

###################################

points = []

def drawPoint(locate_map, point):
    cv2.circle(locate_map, (point[0], point[1]), 15, (255, 0, 0), cv2.FILLED)  #BGR


def drawPoints(locate_map, points):
    for point in points:
        cv2.circle(locate_map, (point[0], point[1]), 2, (255, 255, 255), cv2.FILLED)  #BGR
    cv2.putText(locate_map, f'({(points[-1][0] - 500)/100}m, {-(points[-1][1] - 500)/100}m)', (points[-1][0]+10, points[-1][1]+30), 0, 1, (0, 255, 255))


def directFacing(locate_map, point, facing_point):
    cv2.line(locate_map, (point[0], point[1]),(facing_point[0], facing_point[1]), (0, 0, 255), thickness=1, shift=0)




####################################
def getKeyboardInput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 50
    d = 0
    global x, y, yaw, ang
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
        d = distance_interval
        ang = 90

    if kp.getKey("LEFT"):
        yv = -speed
        yaw -= angular_interval

    elif kp.getKey("RIGHT"):
        yv = speed
        yaw += angular_interval

   

    

    sleep(0.01)

    ang += yaw
    
    x += int(d*math.cos(math.radians(ang)))
    y += int(d*math.sin(math.radians(ang)))

    facing_x = x + int(30 * math.sin(math.radians(yaw)))
    facing_y = y - int(30 * math.cos(math.radians(yaw)))

    
    return [lr, fb, ud, yv, x, y, facing_x, facing_y]

while True:
    vals = getKeyboardInput()
    
    

    #################locate#######################
    locate_map = np.zeros((1000, 1000, 3), np.uint8)

    point = (vals[4], vals[5])
    facing_point = (vals[6], vals[7])
    points.append(point)
    
    drawPoint(locate_map, point)
    drawPoints(locate_map, points)
    directFacing(locate_map, point, facing_point)

    cv2.imshow("locate", locate_map)


    ###############images########################
   
    
    sleep(0.01)