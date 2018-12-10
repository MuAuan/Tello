import sys
import traceback
import tellopy
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy
import time
from time import sleep

def videoCamera(start_time,container,drone,set_time):
    
    frame_skip = 300
    while True:
        for frame in container.decode(video=0):
            if 0 < frame_skip:
                frame_skip = frame_skip - 1
                continue
        
            image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            cv2.imshow('Original', image)
            #cv2.imshow('Canny', cv2.Canny(image, 100, 200))
            cv2.waitKey(1)
            if frame.time_base < 1.0/60:
                time_base = 1.0/60
            else:
                time_base = frame.time_base
            frame_skip = int((time.time() - start_time)/time_base)
            if time.time() - start_time > set_time:
                return
            else:
                continue
                    

def main():
    drone = tellopy.Tello()
    try:
        drone.connect()
        drone.wait_for_connection(60.0)
        drone.takeoff()
        #sleep(10)     
        container = av.open(drone.get_video_stream())
        start_time = time.time()
        videoCamera(start_time,container,drone,10)
        
        drone.down(50)
        sleep(5)
        start_time = time.time()
        videoCamera(start_time,container,drone,10)
        drone.land()
        sleep(5)
        drone.quit()
        cv2.destroyAllWindows()
                    

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
