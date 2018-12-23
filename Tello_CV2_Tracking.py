# -*- coding: utf-8 -*-
import numpy as np
import cv2
import time
from timeit import default_timer as timer
import sys
import traceback
import tellopy
import av
import numpy
from time import sleep
import math as m

def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)

def tracking(drone,d,dx,dy,LB):
    if (d - LB) > 15:
        drone.set_pitch(1)   #drone.pitch = drone.STICK_HOVER + drone.STICK_L
        sleep(1)
    elif (d - LB) < -15:
        drone.set_pitch(-1)   #drone.pitch = drone.STICK_HOVER - drone.STICK_L
        sleep(1)
    elif dx > 80:
        drone.right(5)    #drone.roll = drone.STICK_HOVER + drone.STICK_L
        sleep(1)
    elif dx < -80:
        drone.left(5)    #drone.roll = drone.STICK_HOVER - drone.STICK_L
        sleep(1)
    elif dy > 50:
        drone.up(5)    #drone.thr = drone.STICK_HOVER - drone.STICK_L
        sleep(1)
    elif dy < -50:
        drone.down(5)    #drone.thr = drone.STICK_HOVER + drone.STICK_L
        sleep(1)
    

def key_Operation(drone,key):
    if key == ord('n'):  #n
        drone.down(10)
        sleep(1)
    elif key==ord('u'):  #117:  #u
        drone.up(10)
        sleep(1)
    elif key==ord('h'):  #104:  #h
        drone.left(5)
        sleep(1)
    elif key==ord('j'):  #106:  #j
        drone.right(5)
        sleep(1)
    elif key==ord('b'):  #backward
        drone.backward(5)
        sleep(1)
    elif key==ord('f'):  #forward
        drone.forward(5)
        sleep(1)
    elif key==ord('c'):  #clockwise
        drone.clockwise(10)
        sleep(1)


(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
 
def main():
 
    # Set up tracker.
    # Instead of MIL, you can also use
 
    tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT'] 
    tracker_type = tracker_types[2]
     
    if int(minor_ver) < 3:
        tracker = cv2.Tracker_create(tracker_type)
    else:
        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv2.TrackerGOTURN_create()
        if tracker_type == 'MOSSE':
            tracker = cv2.TrackerMOSSE_create()
        if tracker_type == "CSRT":
            tracker = cv2.TrackerCSRT_create()
 
    # Read video
    #video = cv2.VideoCapture("chaplin.mp4")
    #video = cv2.VideoCapture(0)
    drone = tellopy.Tello()
    drone.connect()
    drone.wait_for_connection(60.0)
        
    container = av.open(drone.get_video_stream()) 
    # Center Cordinates
    CX = 312
    CY = 312
    
    # Reference Distance
    L0 = 100
    S0 = 50176 #224x224

    # Base Distance
    LB = 100

    # 追跡する枠の座標とサイズ
    x = 200
    y = 200
    w = 224
    h = 224
    
    frame_skip=300
    # Define an initial bounding box
    bbox = (x, y, w, h)   #(287, 23, 86, 320)    
    # フレームの取得
    # 追跡する枠を決定
    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip = frame_skip - 1
            continue
        # Uncomment the line below to select a different bounding box
        image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        bbox = cv2.selectROI(image, False)
        print(bbox)
        
        # Initialize tracker with first frame and bounding box
        ok = tracker.init(image, bbox)
        if ok:
            #cv2.destroyWindow("org")
            break
                
    cv2.destroyAllWindows()
    drone.takeoff()
    drone.is_autopilot="True"
    #drone.is_autopilot="False"        
    while True:
        for frame in container.decode(video=0):
            if 0 < frame_skip:
                frame_skip = frame_skip - 1
                continue
            image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            # Start timer
            timer = cv2.getTickCount()
 
            # Update tracker
            ok, bbox = tracker.update(image)
            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
            
            # Draw bounding box
            if ok:
                (x,y,w,h) = (int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3]))
                # Tracking success
                p1 = (x, y)
                p2 = (x + w, y + h)
                cv2.rectangle(image, p1, p2, (255,0,0), 2, 1)
            else :
                # Tracking failure
                cv2.putText(image, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            # Display tracker type on frame
            cv2.putText(image, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
     
            # Display FPS on frame
            cv2.putText(image, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
            
            # Display result
            cv2.imshow("Tracking", image)
            

            key = cv2.waitKey(1)&0xff
            if key == ord('p'):
                drone.is_autopilot="True"
            elif key == ord('s'):
                drone.is_autopilot="False"

            if drone.is_autopilot=="True":
                d = round(L0 * m.sqrt(S0 / (w * h)))
                dx = x + w/2 - CX
                dy = y + h/2 - CY
                print(d,dx,dy,drone.is_autopilot,w,h)
                tracking(drone,d,dx,dy,LB)
            else:
                key_Operation(drone,key)
                print("key=",key,ord('q'))

            if key==ord("q"):
                break
        break
        
    drone.down(50)
    sleep(5)
    drone.land()
    sleep(5)
    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.quit()        


if __name__ == '__main__':
    main()                  