from processing import Processing_Pic 
import cv2 as cv



class Face_detector:
    def __init__(self):
        self.img_processor = Processing_Pic()

    def face_detect(self, img):
        #change gray degree
        gray = self.img_processor.change_gray_degree(img)
        
        face_detect = cv.CascadeClassifier('AI/Face_tracker/model.xml')
        face = face_detect.detectMultiScale(gray)

        for x,y,w,h in face:
            self.img_processor.draw_rectangle(img,x,y,w,h)

        return img




