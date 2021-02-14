#Import Required Libraries
import cv2 as cv
import numpy as np 

#Start the Video Camera
cap = cv.VideoCapture(0)

#Set Parameters for green color in HSV color space
greenLower = (21, 86, 6)
greenUpper = (64, 255, 255)

#Return the frame being captured currently
while(True):
    ret, frame = cap.read()

    if ret==True:

        #Change Color Space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        #Construct a Mask for the color "green", then perform series of dilations and erosions to remove blobs 
        mask = cv.inRange(hsv, greenLower, greenUpper)
        mask = cv.dilate(mask, None, iterations=2)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.bilateralFilter(mask, 9, 75, 75)

        res = cv.bitwise_and(frame, frame, mask=mask)

        #Convert the resultant mask to gray scale so that Hough Circles can be used
        gray = cv.cvtColor(res, cv.COLOR_BGR2GRAY)

        #Blur the frame so as to not detect false circles in the background
        gray = cv.GaussianBlur(gray, (21,21), 0)  

        #Use the Hough Circles Method in opencv to find and store the circles detected in the gray image.  
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1.2, 20, param1=60,  param2=40, minRadius=10, maxRadius=100) 

        #Continue if atleast one circle is found
        if circles is not None:
            
            #Convert the circles stored in circles variable to integers
            #convert (x,y,r) into integers
            circles = np.uint16(np.around(circles))

            #For circles found, Draw the circle and it's center, and also label it 
            for (x, y, r) in circles[0, :]:
                cv.circle(frame, (x,y), r, (255, 0, 0), 3)
                cv.circle(frame, (x,y), 2, (255, 0, 0), 3)
                cv.putText(frame, "Tennis Ball", (x,y), cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255))
        
        #Show the frame to our screen
        cv.imshow("frame", frame)
        cv.imshow("gray", gray)
        cv.imshow("mask", mask)
        cv.imshow("res", res)

        #Break out of the loop if q is pressed
        if cv.waitKey(1) == ord("q"): 
            break
    else:
        break

#Release the camera and close all the windows
cap.release()
cv.destroyAllWindows()
