from djitellopy import tello
import KeyPressModule as kp
import cv2
import numpy as np
from time import sleep
import time
import math
from detect import Face_detector
from kalman_filter import KalmanFilter
from target_tracking import tlbr_to_xyah, xyah_to_tlbr, Processing, judge, cood_to_xywh
from Following import following
from Mapping import drawPoint, drawPoints, directFacing

kp.init()
me = tello.Tello()
me.connect()
me.streamon()
face_detector = Face_detector()

print(me.get_battery())





############### PARAS ###############
img_width = 320
img_height = 320
img_center = [int(img_width/2), int(img_height/2)]


forward_speed = 234
angular_speed = 720
interval = 0.01

distance_interval = forward_speed * interval
angular_interval = angular_speed * interval

x,y = 500,500
ang = 0
yaw = 0

CUDA = False
LOG = False
LOCATE = False

SPEED = 60

recognize_mode = True
track_mode = True
following_mode = False
state = False
mean_tracking, covariance_tracking = None, None
ctrl = None
points = []

#############Hardware######################

if CUDA:
    cv2.cuda.setDevice(0)
    gpu_frame = cv2.cuda_GpuMat()



####################################
def getKeyboardInput(control):
    lr, fb, ud, yv = 0,0,0,0
    speed = SPEED
    d = 0
    global x, y, yaw, ang, recognize_mode, track_mode, following_mode
    
    

    if kp.getKey("a") or control == 'a': 
        lr = -speed
        d = distance_interval
        ang = -180

    elif kp.getKey("d") or control == 'd': 
        lr = speed
        d = -distance_interval
        ang = 180

    if kp.getKey("UP") or control == 'UP': ud = speed
    elif kp.getKey("DOWN") or control == 'DOWN': ud = -speed

    if kp.getKey("w") or control == 'w': 
        fb = speed
        d = distance_interval
        ang = 270

    elif kp.getKey("s") or control == 's': 
        fb = -speed
        d = distance_interval
        ang = 90



    if kp.getKey("LEFT") or control == 'LEFT':
        yv = -speed
        yaw -= angular_interval

    elif kp.getKey("RIGHT") or control == 'RIGHT':
        yv = speed
        yaw += angular_interval



    if kp.getKey("g"): me.land()
    elif kp.getKey("f"): me.takeoff()

    if kp.getKey("q"):
        cv2.imwrite(f'./Images/{time.time()}.jpg', img)
        print("successfully took the photo")
        sleep(0.3)


    if kp.getKey("r"):
        if recognize_mode == False:
            recognize_mode = True
        else:
            recognize_mode = False
        sleep(0.3)

    if kp.getKey("t"):
        if track_mode == False:
            track_mode = True
        else:
            track_mode = False
        sleep(0.3)

    if kp.getKey("m"):
        following_mode = not following_mode


    if kp.getKey("e"):
        me.emergency()


    sleep(interval)
    ang += yaw
    
    x += int(d*math.cos(math.radians(ang)))
    y += int(d*math.sin(math.radians(ang)))

    facing_x = x + int(30 * math.sin(math.radians(yaw)))
    facing_y = y - int(30 * math.cos(math.radians(yaw)))
    return [lr, fb, ud, yv, x, y, facing_x, facing_y, recognize_mode, track_mode]



def form_center(img, center):
    cv2.circle(img, (center[0], center[1]), 5, (0, 255, 0))
    cv2.line(img, (center[0]-10, center[1]),(center[0]+10, center[1]), (0, 255, 0), thickness=1, shift=0)
    cv2.line(img, (center[0], center[1]-10),(center[0], center[1]+10), (0, 255, 0), thickness=1, shift=0)




while True:
    if not following_mode:
        ctrl = None
    vals = getKeyboardInput(control = ctrl)
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    

    #################locate#######################
    if LOCATE:
        locate_map = np.zeros((1000, 1000, 3), np.uint8)

        point = (vals[4], vals[5])
        facing_point = (vals[6], vals[7])
        points.append(point)
        
        drawPoint(locate_map, point)
        drawPoints(locate_map, points)
        directFacing(locate_map, point, facing_point)

        cv2.imshow("locate", locate_map)



    ###############images########################
    img = me.get_frame_read().frame
    if CUDA:
        gpu_frame.upload(img)
        gpu_frame = cv2.cuda.resize(gpu_frame, (img_width, img_height))
        img = gpu_frame.download()
    else:
        img = cv2.resize(img, (img_width, img_height))
        
    form_center(img, img_center)

    #detect mode
    if vals[8]:
        img, cood = face_detector.face_detect(img)
        x1, x2 = cood[0], cood[0] + cood[2]
        y1, y2 = cood[1], cood[1] + cood[3]

        k_filter = KalmanFilter()
        lst = [x1, y1, x2, y2]

        #track mode
        if vals[9]:
            mean_show = [0,0,0,0,0,0,0,0]
            if judge(lst):
                if not state:
                    mean_tracking, covariance_tracking = k_filter.initiate(tlbr_to_xyah(lst))
                    state = True

                else:
                    #update:
                    mean_tracking, covariance_tracking = k_filter.update(mean_tracking, covariance_tracking, tlbr_to_xyah(lst))

                    #predict:
                    mean_tracking, covariance_tracking = k_filter.predict(mean_tracking, covariance_tracking)
                    mean_show = mean_tracking.copy()
            else:
                if state:
                    mean_tracking, covariance_tracking = k_filter.predict(mean_tracking, covariance_tracking)
                    mean_show = mean_tracking.copy()


            box_track = xyah_to_tlbr(mean_show[:4])

            xywh = cood_to_xywh(box_track)

            if LOG:
                print("-------xywh:", xywh)
            cv2.rectangle(img, (int(xywh[0]), int(xywh[1]), int(xywh[2]), int(xywh[3])), color=(0,255,0), thickness=5)
            
            #following mode
            if following_mode:
                # ctrl = following([abs(x2-x1)/2, abs(y2-y1)/2], abs(y2-y1))
                ctrl = following([xywh[0], xywh[1]], xywh[3], img_center)
                sleep(0.3)


        
    cv2.imshow("Image",img)
    
    sleep(interval)