import cv2 as cv


class Processing_Pic:
    
    #coordinate:
    #Eg:x,y,w,h = 100,100,100,100

    def draw_rectangle(self, img, x, y, w, h):
        #draw rectangular:
        cv.rectangle(img, (x, y, w, h), color=(0,0,255), thickness=5)

    #draw circle:
    #cv.circle(img, center=(x, y), radius=100, color=(255,0,0), thickness=3)

    def change_gray_degree(self, img):
        
        #1. change the gray degree
        gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        return gray_img


    def resize_img(self, img):
        #2. resize
        resized_img = cv.resize(img, dsize = (360,360))


        return resized_img









