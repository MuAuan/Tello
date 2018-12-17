from time import sleep
import tellopy
import cv2

def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)

def test():
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        cv2.imshow('test',frame)
        key = cv2.waitKey(1)&0xff
        print("key=",key,ord('q'))
        if key == ord('q'):   #113
            #cv2.destroyAllWindows()
            break
            
    drone = tellopy.Tello()
    #drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

    drone.connect()
    drone.wait_for_connection(60.0)
    drone.takeoff()
    
    while True:
        ret, frame = cap.read()
        cv2.imshow('test',frame)
        key = cv2.waitKey(1)&0xff
        #print("key=",key,"up_u=",ord('u'),"down_n=",ord('n'),"left_h=",ord('h'),"right_j=",ord('j'),"finish_f=",ord('f'))
        print("key=",key,ord('f'))
        
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
        elif key==ord('f'):  #102:  #f
            cv2.destroyAllWindows()
            break
        else:
            continue
     
    drone.down(50)
    sleep(5)
    drone.land()
    sleep(5)
    drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.quit()

if __name__ == '__main__':
    test()
