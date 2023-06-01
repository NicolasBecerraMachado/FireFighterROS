# -*- coding: utf-8 -*-
"""
Created on Thu May 25 15:19:52 2023

@author: lgalm
"""

import time
import cv2
import numpy as np
import imutils

def image_processing(original_image,width,height): 
    """Preprocess the image and obtains the blobs positions """
    objectf = 0
    #Grayscale
    image = cv2.cvtColor(original_image,cv2.COLOR_BGR2HSV)
    x,y,w,h  = 0,0,5,5
    
     ################## RED CANDLE DETECTOR ####################
    lower_1 = np.array([0,75,55])
    upper_1 = np.array([5,255,255])
    red_mask_1 = cv2.inRange(image, lower_1, upper_1)
    
    lower_2 = np.array([170,75,55])
    upper_2 = np.array([179,255,255])
    red_mask_2 = cv2.inRange(image, lower_2, upper_2)
    
    red_mask = red_mask_1 + red_mask_2
    
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    
    red_mask = cv2.dilate(red_mask,kernel,iterations = 4)
    red_mask = cv2.erode(red_mask,kernel,iterations = 2)
    
    
    #find contours
    contours = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    image = cv2.cvtColor(image,cv2.COLOR_HSV2BGR)
    targets=[]
    
    for cnt in contours:
        area=cv2.contourArea(cnt)
        Mmts = cv2.moments(cnt)
        cntX = int(Mmts["m10"]/Mmts["m00"]) #Calculates centroid in X
        cntY = int(Mmts["m01"]/Mmts["m00"]) #Calculates centroid in Y
        centerColor = red_mask[cntY,cntX]
        
        if area > 70 and area < 60000 and centerColor == 255: #Area threshold, adjust to avoid false positives 
            x,y,w,h = cv2.boundingRect(cnt)  #Obtains the dimensions and position of the white blob
            
            #Obtains Coordinates relative to the center of the window
            relX=int(cntX-(width/2))
            relY=-(int(cntY-(height/2)))
            objectf = 1 #indicates theres a target
            
            if((cntX > 0) and (cntX < 480) and (cntY > 0) and (cntY < 640)):
                #Draws a contour around the object found
                cv2.drawContours(image,[cnt],-1,(0,0,255),2)
                #Appends target to targets[]
                targets.append((relX,relY))
                #Marks the centroid of the object with a circle
                cv2.circle(image,(cntX,cntY),4,(0,255,0),-1)
                #puts small circle in the center of the image
                cv2.circle(image,(int(width/2),int(height/2)),3,(0,255,0),-1)
                #Places the coorddinates on top of the target
                cv2.putText(image,"({},{})".format(relX, relY),(cntX-40,cntY-40), cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1)
        
    yOffset = int(h*0.5)
    if y+h+yOffset > 630:
        h=630
    else:
        h = h+yOffset 
    
    if y-yOffset < 10:
        y = 10
    else:
        y=y-yOffset
    
    return image,targets,x,y,w,h, objectf

#Function that turns the original image into RGB and places the bounding box
def AddBoundingBox(original_image, width, height,x,y):
    image = cv2.rectangle(original_image, (x,y), (x+width, y + height), (0,0,0),0)
    return image

#Function that yields a list of all the attributes of a sub image
def Analyze(AR, AG, AB, SR, SG, SB, SSG):
    Data = [AR,AG,AB,SR,SG,SB,SSG]
    return Data


##Image dimensions
width=640
height=480

##Initialization run
cap = cv2.VideoCapture(0)
_, frame = cap.read()
frame = cv2.GaussianBlur(frame, (3,3), 0)

image,targets,X1,Y1,W,H,smth=image_processing(frame,width,height)
image2 = AddBoundingBox(frame, W, H,X1,Y1)
cv2.imshow('window',image)
cv2.imshow('PCView', image2)
cv2.imshow('Region', image2[Y1:Y1+H, X1:X1+W])

######time.sleep(4)
######last_time=time.time()
i = 0
Pics = 0
##Main Loop
while(True):
    #Grabs image from screen
    _, frame = cap.read()
    frame = cv2.GaussianBlur(frame, (3,3), 0)
    #Calls function to obtain bounding boxes
    image,targets,X1,Y1,W,H, smth=image_processing(frame,width,height)
    #Conversion to rgb and acquisition of target image
    image2 = AddBoundingBox(frame, W, H,X1,Y1)
    image3 = image2[Y1:Y1+H, X1:X1+W]  
    #show all images in windows
    cv2.imshow('PCView', image2)
    cv2.imshow('window',image)
    cv2.imshow('Region', image3)
    
    if(Pics): #The selection images are stored to disk if true, 
        #Change the direction to the folder where you wanna store the images
        cv2.imwrite("D:/ProyectoRobotica/Imagenes_limpias/no_fire_var9_"+str(i)+".png", image3)
        i = i+1
        print("Screenshots Taken: {}".format(i))
        time.sleep(0.3)
        Pics = 0
            
    
    #if q is pressed the program closes
    key = cv2.waitKey(25)
    if key == ord('q'):
        cv2.destroyAllWindows()
        break
    elif key == ord('c'): #if c is pressed the target selection is saved as an image
        if Pics == 0:
            Pics = 1
            
        else:
            Pics = 0

cap.release()