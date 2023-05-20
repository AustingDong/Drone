import numpy as np
from kalman_filter import KalmanFilter
import cv2


import time

#x1, y1, x2, y2 -> x, y, a, h

def tlbr_to_xyah(tlbr):
    ret = np.asarray(tlbr).copy()
    ret[2:] -= ret[:2]

    ret[:2] = np.add(ret[:2], ret[2:] / 2, out = ret[:2], casting="unsafe")
    ret[2] /= ret[3]

    return ret




def xyah_to_tlbr(xyah):
    ret = np.asarray(xyah).copy()
    ret[2] *= ret[3]
    ret[:2] = np.subtract(ret[:2], ret[2:] / 2, out = ret[:2], casting="unsafe")
    ret[2] = ret[0] + ret[2]
    ret[3] = ret[1] + ret[3]

    return ret

def cood_to_xywh(cood):
    w = cood[2] - cood[0]
    h = cood[3] - cood[1]
    x = cood[0]
    y = cood[1]
    return x, y, w, h


def bbox_lou(box1, box2):
    b1_x1, b1_y1, b1_x2, b1_y2 = box1[0], box1[1], box1[2], box1[3]
    b2_x1, b2_y1, b2_x2, b2_y2 = box2[0], box2[1], box2[2], box2[3]


    inter_rect_x1 = np.maximum(b1_x1, b2_x1)
    inter_rect_y1 = np.maximum(b1_y1, b2_y1)
    inter_rect_x2 = np.minimum(b1_x2, b2_x2)
    inter_rect_y2 = np.minimum(b1_y2, b2_y2)

    inter_area = np.clip(inter_rect_x2 - inter_rect_x1 + 1, 0, None) * np.clip(inter_rect_y2 - inter_rect_y1 + 1, 0, None)

    b1_area = (b1_x2 - b1_x1 + 1) * (b1_y2 - b1_y1 + 1)
    b2_area = (b2_x2 - b2_x1 + 1) * (b2_y2 - b2_y1 + 1)

    lou = inter_area / (b1_area + b2_area - inter_area + 1e-16)

    return lou

def judge(lst):
    if lst == [0,0,0,0]:
        return False
    return True

def Processing(img, lst, state, mean_tracking, covariance_tracking):
    k_filter = KalmanFilter()
    

    if judge(lst):
        
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
    print("---------box_track_elements:", box_track[0], box_track[1], box_track[2], box_track[3])
    return cood_to_xywh(box_track)