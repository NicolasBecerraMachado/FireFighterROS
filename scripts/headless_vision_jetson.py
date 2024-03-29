#!/usr/bin/env python3
import time
import cv2
import numpy as np
import imutils
import tensorflow as tf
from tensorflow.keras.models import load_model
import os
import socket

#CONSTANTS
src_width=640
src_height=480
currentPan = 100
currentTilt = 70

class Target:
  def __init__(self, cntX, cntY, roi, area, contour, flameCenter, relFlameCenter):
    self.cntX = cntX
    self.cntY = cntY
    self.roi = roi
    self.area = area
    self.contour = contour
    self.relX=int(cntX-(src_width/2))
    self.relY=-(int(cntY-(src_height/2)))
    self.flameCenter = flameCenter
    self.relFlameCenter = relFlameCenter
    self.fireChance = None
    self.currentTarget = False

def adjustBoundingBoxHeight(y,h):
    yOffset = int(h*0.5)
    
    if y+h+yOffset > src_height-10:
        h = src_height-10
    else:
        h = h+yOffset 
    
    if y-yOffset < 10:
        y = 10
    else:
        y = y-yOffset
    
    return y,h

#Keeps adequate candidate targets
def targetFiltering(contours, redMask):
    targets = []
    for contour in contours:
        area=cv2.contourArea(contour)
        Mmts = cv2.moments(contour)
        cntX = int(Mmts["m10"]/Mmts["m00"]) #Calculates centroid in X
        cntY = int(Mmts["m01"]/Mmts["m00"]) #Calculates centroid in Y

        redCenter = redMask[cntY,cntX] == 255
        areaInRange = (area > 350 and area < 60000) #Area threshold, adjust to avoid false positives 
        centerInRange = ((cntX > 0) and (cntX < src_width) and (cntY > 0) and (cntY < src_height))

        if  areaInRange and redCenter and centerInRange: 
            
            x,y,w,h = cv2.boundingRect(contour)  #Obtains the dimensions and position of the white blob
            widthToHeight = w/h
            squareness = area / w*h

            aspectRatioInRange = (widthToHeight > 0.65 and widthToHeight < 1.4)
            squarenessInRange = squareness > 0.7
            
            if aspectRatioInRange and squarenessInRange:

                flameCenter = (cntX,int(cntY-(h/2)))
                relFlameCenter = (-int(flameCenter[0]-(src_width/2)), -(int(flameCenter[1]-(src_height/2))))

                y,h = adjustBoundingBoxHeight(y,h)
                targets.append(Target(cntX, cntY, (x,y,w,h), area, contour, flameCenter, relFlameCenter))
                
    return targets

    

def image_processing(image): 
    objectf = 0
    #HSV Conversion
    imageHSV = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    
    ################## RED CANDLE DETECTOR ####################
    lower_1 = np.array([0,55,55])
    upper_1 = np.array([6,255,255])
    redMask_1 = cv2.inRange(imageHSV, lower_1, upper_1)
    
    lower_2 = np.array([170,55,55])
    upper_2 = np.array([179,255,255])
    redMask_2 = cv2.inRange(imageHSV, lower_2, upper_2)
    
    redMask = redMask_1 + redMask_2
    
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    
    redMask = cv2.dilate(redMask,kernel,iterations = 4)
    redMask = cv2.erode(redMask,kernel,iterations = 2)
    
    
    #find contours
    contours = cv2.findContours(redMask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    targets=[]
    
    targets = targetFiltering(contours, redMask)
    
    return targets


#Classifies image as fire or no fire
def imageClassification(img, model):
    
    fire = False
    fireChance = 0

    resized = tf.image.resize(img, (256,128))
    fireChance = 1 - model.predict(np.expand_dims(resized/255, 0))
    print(fireChance)
    if fireChance > 0.8:
        fire = True
    
    return fire, fireChance

def rankTargets(targets):

    globalScores = []
    areaScores = []
    positionScores = []

    for target in targets:

        areaScores.append(target.area)
        positionScores.append(abs(target.relX))

    areaScores = [float(areaScore)/max(areaScores) for areaScore in areaScores]
    positionScores = [1-(float(positionScore)/(max(positionScores)+0.001)) for positionScore in positionScores]
    
    for i in range(len(areaScores)):
        globalScores.append(areaScores[i]*0.7 + positionScores[i]*0.3)
    
    rankedTargets = dict(zip(targets, globalScores))

    return rankedTargets


def fireDetection(original_image, targets, model):

    for target in targets:
        #Conversion to rgb and acquisition of target image
        roi = original_image[target.roi[1]:target.roi[1]+target.roi[3], \
                            target.roi[0]:target.roi[0]+target.roi[2]].copy()

        roi = cv2.cvtColor(roi,cv2.COLOR_BGR2RGB)
        
        #Candidate Image Classification
        fire, fireChance = imageClassification(roi, model)

        target.fireChance = fireChance
        print("Fire: {}".format(fire))

        if fire:
            target.currentTarget = True
            return target

    return None

def getAngleDelta(axis, angle):
    
    angleToScreenRatio = 0
    if axis == 0:
        angleToScreenRatio = angle / (src_width/2)
    else:
        angleToScreenRatio = angle / (src_height/2)
    
    
    if angleToScreenRatio > 0.5:
        return 7
    elif angleToScreenRatio > 0.2:
        return 4
    elif angleToScreenRatio > 0.05:
        return 1
    elif angleToScreenRatio > -0.05:
        return 0
    elif angleToScreenRatio > -0.2:
        return -1
    elif angleToScreenRatio > -0.5:
        return -4
    else:
        return -7

def sendFireData(targets, s):
    global currentPan
    global currentTilt

    if len(targets) > 0:
        currentPan = currentPan + getAngleDelta(0,targets[0].relFlameCenter[0])
        currentTilt = currentTilt + getAngleDelta(1,targets[0].relFlameCenter[1])

        if currentPan > 180:
            currentPan = 180
        elif currentPan < 0:
            currentPan = 0

        if currentTilt > 180:
            currentTilt = 180
        elif currentTilt < 0:
            currentTilt = 0

        if s is not None:
            output = "{},{}".format(abs(int(currentPan)), abs(int(currentTilt)))

            fireHomingEnabled = "1"
            fireInWaterRange = "1" if targets[0].area > 9000 else "0"
            fireAngle = int(targets[0].relFlameCenter[0] / src_width * 55)
            output += ",{},{},{}".format(fireHomingEnabled,fireInWaterRange,fireAngle)
            s.send(bytes(output, "utf-8"))
    else:
        currentPan = 100
        currentTilt = 70

        if s is not None:
            output = "{},{}".format(abs(int(currentPan)), abs(int(currentTilt)))

            fireHomingEnabled = "0"
            fireInWaterRange = "0"
            fireAngle = "inf"
            output += ",{},{},{}".format(fireHomingEnabled,fireInWaterRange,fireAngle)
            s.send(bytes(output, "utf-8"))
    

    


if __name__=="__main__":
    
    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s.bind((socket.gethostname(),1234))
    #s.listen(4)
    
    gpus = tf.config.list_physical_devices('GPU')
    tf.config.experimental.set_memory_growth(gpus[0],True)

    model = load_model(os.path.join('../models','fireDetection.h5'))
    cap = cv2.VideoCapture(0)
    _, original_image = cap.read()
    src_width=original_image.shape[1]
    src_height=original_image.shape[0]
    print("Image Dimensions: {}x{}".format(src_width,src_height))

    #clientSocket, _ = s.accept()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((socket.gethostname(),1234))

    fireTarget = ""
    ##Main Loop
    while(True):
        i = 0
        fire = False
        #Grabs image from screen
        _, original_image = cap.read()
        original_image = cv2.GaussianBlur(original_image, (3,3), 0)
        image = original_image.copy()

        #Calls function to obtain bounding boxes
        targets = image_processing(image)

        #selects the highest priority target
        rankedTargets = rankTargets(targets)

        if i >= 5:
            #Gets ROIs and classifies them
            fireTarget = fireDetection(original_image, rankedTargets, model)
            i = 0

        #Send message to pan tilt control
        sendFireData(targets, s)

        
        i+=1

        time.sleep(25/1000)
        

    #clientSocket.close()
    cap.release()
