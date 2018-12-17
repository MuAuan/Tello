from time import sleep
import tellopy
import cv2
import av
import numpy

def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)

def test():
    #cap = cv2.VideoCapture(0)
    drone = tellopy.Tello()
    #drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

    drone.connect()
    drone.wait_for_connection(60.0)
    container = av.open(drone.get_video_stream())
    x = 200
    y = 200
    w = 224
    h = 224
    track_window = (x, y, w, h)
    frame_skip=300
    
    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip = frame_skip - 1
            continue
        image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        cv2.imshow("org",image)
        k = cv2.waitKey(1)&0xff
        if k == ord('q'):
            #cv2.destroyWindow("org")
            break
            
    drone.takeoff()
    
    while True:
        for frame in container.decode(video=0):
            if 0 < frame_skip:
                frame_skip = frame_skip - 1
                continue
            
            image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
        
            cv2.imshow("org",image)
            key = cv2.waitKey(1)&0xff
            print("key=",key,ord('q'))
        
            if key == ord('n'):  #n
                drone.down(10)
                sleep(5)
            elif key==ord('u'):  #117:  #u
                drone.up(10)
                sleep(5)
            elif key==ord('h'):  #104:  #h
                drone.left(3)
                sleep(1)
            elif key==ord('j'):  #106:  #j
                drone.right(3)
                sleep(1)
            elif key==ord('b'):  #backward
                drone.backward(3)
                sleep(1)
            elif key==ord('f'):  #forward
                drone.forward(3)
                sleep(1)
            elif key==ord('c'):  #clockwise
                drone.clockwise(10)
                sleep(1)    
            elif key==ord('q'):  #quit
                cv2.destroyAllWindows()
                break
            else:
                continue
        break
    drone.down(50)
    sleep(5)
    drone.land()
    sleep(5)
    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.quit()

if __name__ == '__main__':
    test()
