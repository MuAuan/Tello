# -*- coding: utf-8 -*-
 
import numpy as np
import cv2
import time
from timeit import default_timer as timer
import sys
import traceback
import tellopy
import av
#import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy
from time import sleep
import math as m

import keras
from keras.models import Model, Input
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.applications.vgg16 import VGG16, preprocess_input, decode_predictions
from keras.preprocessing import image
#from SpatialPyramidPooling import SpatialPyramidPooling

def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)

def cv_fourcc(c1, c2, c3, c4):
        return (ord(c1) & 255) + ((ord(c2) & 255) << 8) + \
            ((ord(c3) & 255) << 16) + ((ord(c4) & 255) << 24)

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

        
def main():     
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
    track_window = (x, y, w, h)
    frame_skip=300
    
    # フレームの取得
    # 追跡する枠を決定
    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip = frame_skip - 1
            continue
        image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        cv2.imshow("org",image)
        roi = image[y:y+h, x:x+w]
        cv2.imshow("roi",roi)
        k = cv2.waitKey(1)
        if k == ord('q'):
            txt=yomikomi(roi)
            print(txt)
            cv2.destroyWindow("org")
            break

       
    # 追跡する枠の内部を切り抜いてHSV変換
    hsv_roi =  cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv",hsv_roi)
    ## マスク画像の生成
    img_mask = cv2.inRange(hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
    ## 正規化するためのヒストグラムの生成 
    roi_hist = cv2.calcHist([hsv_roi], [0], img_mask, [180], [0,180])
    ## ノルム正規化
    cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX) 
 
    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

    accum_time = 0
    curr_fps = 0
    fps = "FPS: ??"
    prev_time = timer()
    start_time=prev_time
    
    OUT_FILE_NAME = "meanshiftObjDet_TR.mp4"
    FRAME_RATE=8
    out = cv2.VideoWriter(OUT_FILE_NAME, \
                  cv_fourcc('M', 'P', '4', 'V'), \
                  FRAME_RATE, \
                  (w, h), \
                  True)
    cv2.destroyAllWindows()
    drone.takeoff()
    #drone.is_autopilot="True"
    drone.is_autopilot="False"
    while True:
        for frame in container.decode(video=0):
            if 0 < frame_skip:
                frame_skip = frame_skip - 1
                continue
            image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
         
            # フレームをHSV変換する
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # 上で計算したヒストグラムを特徴量として、画像の類似度を求める
            dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180], 1)
 
            # 物体検出する
            ret, track_window = cv2.meanShift(dst, track_window, term_crit)
            #ret, track_window = cv2.CamShift(dst, track_window, term_crit)
 
            # 物体検出で取得した座標を元のフレームで囲う
            x,y,w,h = track_window
            img_dst = cv2.rectangle(image, (x,y), (x+w, y+h), 255, 2)
           
            cv2.putText(img_dst, txt, (x+3,y+10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), 1)  #dst txt
            cv2.imshow('SHOW MEANSHIFT IMAGE', img_dst)
            
            key = cv2.waitKey(1)&0xff
            
            if key == ord('p'):
                drone.is_autopilot="True"
            elif key == ord('s'):
                drone.is_autopilot="False"
            #else:
            #    continue
                      
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
                else:
                    img_dst = cv2.resize(img_dst, (int(h), w))
                    out.write(img_dst)
                    continue
                    
        break
        
    drone.down(50)
    sleep(5)
    drone.land()
    sleep(5)
    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.quit()
             
    
def yomikomi(img):
    batch_size = 2
    num_classes = 1000
    img_rows, img_cols=img.shape[0],img.shape[1]
    input_tensor = Input((img_rows, img_cols, 3))

    # 学習済みのVGG16をロード
    # 構造とともに学習済みの重みも読み込まれる
    model = VGG16(weights='imagenet', include_top=True, input_tensor=input_tensor)
    model.summary()
    
    # 引数で指定した画像ファイルを読み込む
    # サイズはVGG16のデフォルトである224x224にリサイズされる
    # 読み込んだPIL形式の画像をarrayに変換
    x = image.img_to_array(img)
    x = np.expand_dims(x, axis=0)
    #preds = model.predict(preprocess_input(x))
    preds = model.predict(x)
    results = decode_predictions(preds, top=1)[0]
    return str(results[0][1])
                
if __name__ == '__main__':
    main()            