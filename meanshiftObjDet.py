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

import keras
from keras.models import Model, Input
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten
from keras.applications.vgg16 import VGG16, preprocess_input, decode_predictions
from keras.preprocessing import image
#from SpatialPyramidPooling import SpatialPyramidPooling

def cv_fourcc(c1, c2, c3, c4):
        return (ord(c1) & 255) + ((ord(c2) & 255) << 8) + \
            ((ord(c3) & 255) << 16) + ((ord(c4) & 255) << 24)

def main():     
    drone = tellopy.Tello()
    drone.connect()
    drone.wait_for_connection(60.0)
    #drone.takeoff()
        
    container = av.open(drone.get_video_stream()) 
    #cap = cv2.VideoCapture(0)
    
    # 追跡する枠の座標とサイズ
    x = 200
    y = 200
    w = 224
    h = 224
    track_window = (x, y, w, h)
    frame_skip=300
    
    # フレームの取得
    #ret,frame = cap.read()
    # 追跡する枠を決定
    #while True:
    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip = frame_skip - 1
            continue
        #start_time = time.time()
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
    
    OUT_FILE_NAME = "meanshiftObjDet_result.mp4"
    FRAME_RATE=8
    out = cv2.VideoWriter(OUT_FILE_NAME, \
                  cv_fourcc('M', 'P', '4', 'V'), \
                  FRAME_RATE, \
                  (w, h), \
                  True)

    
    drone.takeoff()
    
    while(True):
        #ret, frame = cap.read()
        for frame in container.decode(video=0):
            if 0 < frame_skip:
                frame_skip = frame_skip - 1
                continue
            start_time = time.time()
            image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        
 
        #if ret == True:
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
            cv2.putText(img_dst, txt, (x+3,y+10), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,0), 1)
            #cv2.imshow('SHOW MEANSHIFT IMAGE', img_dst)
            curr_time = timer()
            exec_time = curr_time - prev_time
            prev_time = curr_time
            accum_time = accum_time + exec_time
            curr_fps = curr_fps + 1
            if accum_time > 1:
                accum_time = accum_time - 1
                fps = "FPS: " + str(curr_fps)
                curr_fps = 0
            cv2.putText(img_dst, fps, (x+3,y+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1)
            #cv2.putText(img_dst, str(dst), (x+3,y+10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,0,0), 1)
            
            cv2.imshow('SHOW MEANSHIFT IMAGE', img_dst)
            img_dst = cv2.resize(img_dst, (int(h), w))
            out.write(img_dst)
            # qを押したら終了。
            k = cv2.waitKey(1)
            if k == ord('q'):
                drone.land()
                sleep(5)
                drone.quit()
                cv2.destroyWindow('SHOW MEANSHIFT IMAGE')
                break
            else:
                break
                
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