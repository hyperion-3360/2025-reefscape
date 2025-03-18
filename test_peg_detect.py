
import cv2 
import numpy as np 
 
def click_event(event, x, y, flags, params): 

    # checking for left mouse clicks 
    if event == cv2.EVENT_LBUTTONDOWN: 
  
        # displaying the coordinates 
        # on the Shell 
        print(x, ' ', y) 
  
        # displaying the coordinates 
        # on the image window 
        font = cv2.FONT_HERSHEY_SIMPLEX 
        cv2.putText(img_gray, str(x) + ',' +
                    str(y), (x,y), font, 
                    1, (255, 0, 0), 2) 
        cv2.imshow('Gray', img_gray ) 
  
    # checking for right mouse clicks      
    if event==cv2.EVENT_RBUTTONDOWN: 
  
        # displaying the coordinates 
        # on the Shell 
        print(x, ' ', y) 
  
        # displaying the coordinates 
        # on the image window 
        font = cv2.FONT_HERSHEY_SIMPLEX 
        h = img_gray[y, x, 0]
        s = img_gray[y, x, 1]
        v = img_gray[y, x, 2]
        cv2.putText(img_gray, str(h) + ',' + str(s) + ',' + str(v), 
                    (x,y), font, 1, 
                    (255, 255, 0), 2) 
        cv2.imshow('Gray', img_gray) 
 
img = cv2.imread('./peg5.png')

img = cv2.resize(img, (640,480))

cv2.imshow('Original', img)
cv2.waitKey(0)
 
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 

#img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# Blur the image for better edge detection
cv2.imshow('Gray', img_gray)
cv2.setMouseCallback('Gray', click_event) 
cv2.waitKey(0)

# Threshold of blue in HSV space 
lower_violet = np.array([150, 60, 70]) 
upper_violet = np.array([175, 165, 130]) 
  
# preparing the mask to overlay 
mask = cv2.inRange(img_gray, lower_violet, upper_violet) 

cv2.imshow('Mask', mask)
cv2.waitKey(0)

contours, hierarchy = cv2.findContours(mask,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

print("Number of Contours found = " + str(len(contours))) 

c = max(contours, key = cv2.contourArea)
x,y,w,h = cv2.boundingRect(c)

# draw the biggest contour (c) in green
cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

cv2.imshow('Bounding box', img)
cv2.waitKey(0)


#img_blur = cv2.GaussianBlur(img_gray, (3,3), 0) 
# 
#ret,thresh = cv2.threshold(img_blur,50,150,0)
#cv2.imshow('Threshold', thresh)
#contours,hierarchy = cv2.findContours(thresh, 1, 2)
# 
#cnt = contours[0]
#rect = cv2.minAreaRect(cnt)
#box = cv2.boxPoints(rect)
#box = np.intp(box)
#
#cv2.drawContours(img,[box],0,(0,0,255),2)
#cv2.waitKey(0)
 
cv2.destroyAllWindows()
