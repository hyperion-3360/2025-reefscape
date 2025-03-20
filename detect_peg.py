
import cv2 
import numpy as np 
import argparse

# x1, y1 of the ROI
x1, y1, x2, y2 = -1, -1, -1, -1

img_hsv = None
 
def click_event(event, x, y, flags, params): 
    global x1, y1, x2, y2, img_hsv

    # checking for left mouse clicks 
    if event == cv2.EVENT_LBUTTONDOWN: 
        x1 = x
        y1 = y
    if event == cv2.EVENT_LBUTTONUP:
        x2 = x
        y2 = y
        print( x1, y1, x2, y2)
        cv2.rectangle(img_hsv, (x1, y1),(x, y),(0, 255, 0),2)
        cv2.imshow('Gray', img_hsv)

    # checking for right mouse clicks      
    if event==cv2.EVENT_RBUTTONDOWN: 
  
        # displaying the coordinates 
        # on the Shell 
        print(x, ' ', y) 
  
        # displaying the coordinates 
        # on the image window 
        font = cv2.FONT_HERSHEY_SIMPLEX 
        h = img_hsv[y, x, 0]
        s = img_hsv[y, x, 1]
        v = img_hsv[y, x, 2]
        print(h , ' ', s, ' ', v)

def process_peg_image(image_File_name, lower_violet_hsv, upper_violet_hsv):
    global img_hsv
    img = cv2.imread(image_File_name)

    img = cv2.resize(img, (640,480))

    cv2.imshow('Original', img)
    cv2.waitKey(0)
    
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 

    #img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Blur the image for better edge detection
    cv2.imshow('Gray', img_hsv)
    cv2.setMouseCallback('Gray', click_event) 
    cv2.waitKey(0)

    # rectangle drawn so use the ROI to analyse the hsv
    if( x1 != -1 and y1 != -1 and x2 != -1 and y2 != -1):
        # get the portion of the image in the rectangle
        roi = img_hsv[y1:y2, x1:x2]
        #split in HSV planes    
        hsv_planes = cv2.split(roi)
        lower_hsv = []
        upper_hsv = []
        # for each plane find that value which is greater than 90% of the pixels in the plane
        for plane in hsv_planes:
            hist = cv2.calcHist([plane], [0], None, [256], [0, 256])
            hist = hist / hist.sum()
            lower10 = 0
            upper90 = 0
            for i in range(255):
                upper90 += hist[i]
                if upper90 > 0.9:
                    upper_hsv.append(i)
                    break
            for i in range(255):
                lower10 += hist[i]
                if lower10 > 0.1:
                    lower_hsv.append(i)
                    break
        print("Lower HSV:")
        print( lower_hsv)

        print("Upper HSV:" )
        print(upper_hsv)

    # preparing the mask to overlay 
    mask = cv2.inRange(img_hsv, lower_violet_hsv, upper_violet_hsv) 

    contours, hierarchy = cv2.findContours(mask,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) 

    c = max(contours, key = cv2.contourArea)
    x,y,w,h = cv2.boundingRect(c)

    # draw the biggest contour (c) in green
    cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)

    cv2.imshow('Bounding box', img)
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
# using argparse to get the image file name
    parser = argparse.ArgumentParser()
    parser.add_argument("image_file", help="The image file to process") 
    args = parser.parse_args()
    image_File_name = args.image_file

    # Threshold of blue in HSV space 
    lower_violet = np.array([158, 72, 168]) 
    upper_violet = np.array([166, 122, 247]) 

    process_peg_image(image_File_name, lower_violet, upper_violet)
    