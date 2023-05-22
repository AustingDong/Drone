import cv2

def drawPoint(locate_map, point):
    cv2.circle(locate_map, (point[0], point[1]), 10, (255, 0, 0), cv2.FILLED)  #BGR


def drawPoints(locate_map, points):
    for point in points:
        cv2.circle(locate_map, (point[0], point[1]), 1, (255, 255, 255), cv2.FILLED)  #BGR
    cv2.putText(locate_map, f'({(points[-1][0] - 500)/100}m, {-(points[-1][1] - 500)/100}m)', (points[-1][0]+10, points[-1][1]+30), 0, 1, (0, 255, 255))


def directFacing(locate_map, point, facing_point):
    cv2.line(locate_map, (point[0], point[1]),(facing_point[0], facing_point[1]), (0, 0, 255), thickness=1, shift=0)
