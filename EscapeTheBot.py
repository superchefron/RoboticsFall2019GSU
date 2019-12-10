import threading
from easygopigo3 import EasyGoPiGo3
import sys
import signal
from time import sleep
from gpiozero import LightSensor
from gpiozero import LED

MX_DISTANCE = 2300
MN_DISTANCE = 150
TOOCLOSE_DISTANCE = 50
NO_OBSTACLE = 3000
ERROR = 0
MX_SPEED = 300
MN_SPEED = 100

robot_operating = True

def signal_handler(signal, frame):
    global robot_operating
    print("CTRL-C combination pressed")
    robot_operating = False
    
def videoStream():
    cap = cv2.VideoCapture(0)
    while(True):
        ret, frame = cap.read()
        frame = cv2.flip(frame, -1)
        
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    

def objectAvoidance():
    sleep(3)
    try:
        gopigo3 = EasyGoPiGo3()
        distance_sensor = gopigo3.init_distance_sensor()
    except IOError as msg:
        print("GoPiGo3 robot and sensors are not detected")
        sys.exit(1)
        
    gopigo3_stationary = True
    global robot_operating
    
    while robot_operating:
        current_distance = distance_sensor.read_mm()
        sleep(0.02)
        determined_speed = 0
        
        if current_distance < MN_DISTANCE and current_distance > TOOCLOSE_DISTANCE:
            gopigo3_stationary = True
            gopigo3.stop()
            sleep(0.02)
        elif current_distance < TOOCLOSE_DISTANCE:
            gopigo3_stationary = False
            percent_speed = float(current_distance - MN_DISTANCE) / (MX_DISTANCE - MN_DISTANCE)
            determined_speed = MN_SPEED + (MX_SPEED - MN_SPEED) * percent_speed
            gopigo3.set_speed(determined_speed)
            gopigo3.drive_cm(-10,True)
        else:
            gopigo3_stationary = False
            if current_distance == NO_OBSTACLE:
                determined_speed = MX_SPEED
            else:
                percent_speed = float(current_distance - MN_DISTANCE) / (MX_DISTANCE - MN_DISTANCE)
                determined_speed = MN_SPEED + (MX_SPEED - MN_SPEED) * percent_speed
            gopigo3.set_speed(determined_speed)
            #gopigo3.forward()
        
    gopigo3.stop()

ldr_right = LightSensor(24)
ldr_left = LightSensor(4)
ldr_right.threshold = .05
ldr_left.threshold = .05

def blinkers():
    gpg = EasyGoPiGo3()
    global robot_operating
    while (robot_operating):
        if (ldr_right.light_detected and not ldr_left.light_detected):
            gpg.set_led(gpg.LED_BLINKER_RIGHT, 0)
            gpg.set_led(gpg.LED_BLINKER_LEFT, 255)
            gpg.set_led(gpg.LED_EYE_LEFT, 0)
            gpg.set_led(gpg.LED_EYE_RIGHT, 0)
            sleep(0.02)
        elif (ldr_left.light_detected and not ldr_right.light_detected):
            gpg.set_led(gpg.LED_BLINKER_RIGHT, 255)
            gpg.set_led(gpg.LED_BLINKER_LEFT, 0)
            gpg.set_led(gpg.LED_EYE_LEFT, 0)
            gpg.set_led(gpg.LED_EYE_RIGHT, 0)
            sleep(0.02)
        elif (ldr_right.light_detected and ldr_left.light_detected):
            gpg.set_led(gpg.LED_BLINKER_RIGHT, 0)
            gpg.set_led(gpg.LED_BLINKER_LEFT, 0)
            gpg.set_led(gpg.LED_EYE_LEFT, 255)
            gpg.set_led(gpg.LED_EYE_RIGHT, 255)
            sleep(0.02)
        else:
            gpg.set_led(gpg.LED_BLINKER_RIGHT, 0)
            gpg.set_led(gpg.LED_BLINKER_LEFT, 0)
            gpg.set_led(gpg.LED_EYE_LEFT, 0)
            gpg.set_led(gpg.LED_EYE_RIGHT, 0)
            sleep(0.02)
            
        
def movement():
    gopigo3n = EasyGoPiGo3()
    while(True):
        if (ldr_right.light_detected and not ldr_left.light_detected):
            #print("Right LDR detected!")
            gopigo3n.left()
            sleep(0.02)
        elif (ldr_left.light_detected and not ldr_right.light_detected):
            #print("Left LDR detected!")
            gopigo3n.right()
            sleep(0.02)
        elif (ldr_right.light_detected and ldr_left.light_detected):
            #print("No LDRs detected!")
            gopigo3n.stop()
            sleep(0.02)
        else:
            #print("Both LDR detected!")
            gopigo3n.forward()
            sleep(0.02)
             
def main_task():
    t1 = threading.Thread(target=objectAvoidance)
    t2 = threading.Thread(target=movement)
    t3 = threading.Thread(target=blinkers)
    
    t1.start()
    t2.start()
    t3.start()
    
    t1.join()
    t2.join()
    t3.join()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main_task()