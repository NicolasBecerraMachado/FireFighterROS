import time
import cv2
import numpy as np
import imutils
import tensorflow as tf
from tensorflow.keras.models import load_model
import os
#import rospy
#from std_msgs.msg import UInt8

#CONSTANTS
src_width=640
src_height=480

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
        areaInRange = (area > 350 and area < 600000) #Area threshold, adjust to avoid false positives 
        centerInRange = ((cntX > 0) and (cntX < src_width) and (cntY > 0) and (cntY < src_height))

        if  areaInRange and redCenter and centerInRange: 
            
            x,y,w,h = cv2.boundingRect(contour)  #Obtains the dimensions and position of the white blob
            widthToHeight = w/h
            squareness = area / w*h

            aspectRatioInRange = (widthToHeight > 0.5 and widthToHeight < 2)
            squarenessInRange = squareness > 0.5
            
            if aspectRatioInRange and squarenessInRange:

                flameCenter = (cntX,int(cntY-(h/2)))
                relFlameCenter = (int(flameCenter[0]-(src_width/2)), (int(flameCenter[1]-(src_height/2))))

                y,h = adjustBoundingBoxHeight(y,h)
                targets.append(Target(cntX, cntY, (x,y,w,h), area, contour, flameCenter, relFlameCenter))
                
    return targets


def drawTarget(image, target):
    #Draws a contour around the object found
    cv2.drawContours(image,[target.contour],-1,(0,0,255),2)

    #Marks the centroid of the object with a circle
    #cv2.circle(image,(target.cntX,target.cntY),4,(0,255,0),-1)

    #Places the coorddinates on top of the target
    cv2.putText(image,"({},{})".format(target.relFlameCenter[0], target.relFlameCenter[1]),(target.roi[0],target.roi[1]-8), \
                cv2.FONT_HERSHEY_SIMPLEX,0.4,(0,0,255),1)

    #Marks the centroid of the object with a circle
    cv2.circle(image,(target.flameCenter[0], target.flameCenter[1]),4,(0,0,255),4)

    

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

    for target in targets:
        drawTarget(image, target)

    #puts small circle in the center of the image
    cv2.circle(image,(int(src_width/2),int(src_height/2)),3,(0,255,0),-1)
    
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

#Function that turns the original image into RGB and places the bounding box
def AddBoundingBoxes(image, targets):
    for target in targets:
        x = target.roi[0]
        y = target.roi[1]
        w = target.roi[2]
        h = target.roi[3]

        color = (0,255,0)
        if target.currentTarget:
            color = (0,0,255)
        
        cv2.line(image, (int(src_width/2),int(src_height/2)), (target.flameCenter[0], target.flameCenter[1]), (0,255,0), 2)
        cv2.rectangle(image, (x,y), (x+w, y + h), color,2)

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


def aimAtTarget(target, pubPan, pubTilt, message):
    
    if target is not None:
        currentPan = currentPan + (target.relFlameCenter[0] * 0.7)
        currentTilt = currentTilt + (target.relFlameCenter[1] * 0.7)

    else:
        currentPan = currentPan + (135 - currentPan) * 0.7
        currentTilt = currentTilt + (90 - currentPan) * 0.7

    #message.data = int(currentPan)
    #pubPan.publish(message)
    #message.data = int(currentTilt)
    #pubTilt.publish(message)
    



if __name__=="__main__":
    #rospy.init_node('vision',anonymous=True)

    #define ros publishers
    #pubPan = rospy.Publisher('arduino/pan',UInt8,queue_size=10)
    #pubTilt = rospy.Publisher('arduino/tilt',UInt8,queue_size=10)
    #message = UInt8()

    #model = load_model(os.path.join('../models','fireDetection.h5'))
    cap = cv2.VideoCapture(0)
    _, original_image = cap.read()

    currentPan = 135
    currentTilt = 90
    src_width=original_image.shape[1]
    src_height=original_image.shape[0]

    ##Main Loop
    while(True):

        fire = False
        #Grabs image from screen
        _, original_image = cap.read()
        original_image = cv2.GaussianBlur(original_image, (3,3), 0)
        image = original_image.copy()
	
        #Calls function to obtain bounding boxes
        targets = image_processing(image)

        #selects the highest priority target
        rankedTargets = rankTargets(targets)

        #Gets ROIs and classifies them
        #fireTarget = fireDetection(original_image, rankedTargets, model)

        #Marks targets on screen
        AddBoundingBoxes(image, rankedTargets)

        #Points at target
        #aimAtTarget(targets[0], pubPan, pubTilt, message)

        #show all images in windows
        cv2.imshow('image', image)
                
        #if q is pressed the program closes
        key = cv2.waitKey(25)
        if key == ord('q'):
            cv2.destroyAllWindows()
            break

    cap.release()
