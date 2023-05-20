from djitellopy import tello
import KeyPressModule as kp
import cv2
from time import sleep
import time


kp.init()
me = tello.Tello()
me.connect()
me.streamon()
print(me.get_battery())

global img
def getKeyboardInput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 50

    if kp.getKey("a"): lr = -speed
    elif kp.getKey("d"): lr = speed

    if kp.getKey("UP"): ud = speed
    elif kp.getKey("DOWN"): ud = -speed

    if kp.getKey("w"): fb = speed
    elif kp.getKey("s"): fb = -speed

    if kp.getKey("LEFT"): yv = -speed
    elif kp.getKey("RIGHT"): yv = speed

    if kp.getKey("g"): me.land()
    elif kp.getKey("f"): me.takeoff()

    if kp.getKey("q"):
        cv2.imwrite(f'./TELLO/Images/{time.time()}.jpg', img)
        print("successfully took the photo")
        sleep(0.3)

    return [lr, fb, ud, yv]

while True:
    vals = getKeyboardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
    img = me.get_frame_read().frame
    img = cv2.resize(img, (360, 240))
    cv2.imshow("Image",img)
    sleep(0.01)