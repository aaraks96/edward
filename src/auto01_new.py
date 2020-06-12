import RPi.GPIO as gpio
import numpy as np
import time
import matplotlib.pyplot as plt
import serial 
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import math


# initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640,480))
time.sleep(0.1)
image_count = 0

angle = 0  
pos = (0, 0)  

all_x = list()
all_y = list()
all_angle = list()

f = open("edward_pos_data.txt", "w+")



ser = serial.Serial('/dev/ttyUSB0',9600)
for i in range(15):
    line = ser.readline()
    
#initialize gpio pins to control servo motor
gpio.setmode(gpio.BOARD)
gpio.setup(36,gpio.OUT)
pwm = gpio.PWM(36,50)
duty = 5.5
pwm.start(duty)

#pin allocation for ultrasonic sensor
trig = 16
echo = 18

def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT) #IN1
    gpio.setup(33, gpio.OUT) #IN2
    gpio.setup(35, gpio.OUT) #IN3
    gpio.setup(37, gpio.OUT) #IN4
    gpio.setup(36, gpio.OUT) #ultrasonic
    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
    #set all pins to low
    gpio.output(31, False)
    gpio.output(33, False)
    gpio.output(35, False)
    gpio.output(37, False)
    gpio.cleanup()
    
    
def get_target(pos, angle):
    full_circle_count = 0
   
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        TargetObtained = False
        
        while TargetObtained != True:
        # grab the current frame
            image = frame.array
            # show the frame to our screen
            image = cv2.flip(image, -1)
            #cv2.imshow("output", image)
            global image_count
            image_count+=1
            cv2.imwrite("ed_see_"+str(image_count)+".jpg",image)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            #cv2.imshow("hsv", hsv_image)

            mask = cv2.inRange(hsv_image, lower_mask, upper_mask)
            #cv2.imshow("mask", mask)
            #cv2.waitKey(0)
            _,cnts,h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(cnts)==0:
                rawCapture.truncate(0)
                if full_circle_count >24:
                    print("target not obtained in full circle, going forward!")
                    pos, angle = forward(0.2, pos, angle)
                    
                else:
                    full_circle_count= full_circle_count+1
                    print("target not obtained, turning left")                    
                    pos, angle = left(15, pos, angle)
                
                print("Current vehicle orientation(line 83): ",angle)
                TargetObtained = True
            else:
                rawCapture.truncate(0)
                print("Target obtained")
                TargetObtained = True
                cnts_sorted = sorted(cnts, key=cv2.contourArea)
                area = cv2.contourArea(cnts_sorted[-1])
                
                if area <70.0:
                    if full_circle_count >24:
                        print("target not obtained in full circle, going forward!")
                        pos, angle = forward(0.2, pos, angle)
                    
                    else:
                        full_circle_count= full_circle_count+1
                        print("target not obtained, turning left")                    
                        pos, angle = left(15, pos, angle)
                    
                    print("Current vehicle orientation(line 93): ",angle)
                    continue
                    
                hull = cv2.convexHull(cnts_sorted[-1])
                (x,y),radius = cv2.minEnclosingCircle(hull)
                print("Circle center", (x,y))
                circled_image = cv2.circle(image,(int(x),int(y)),int(radius),(0,0,255),4)
                cv2.line(circled_image, (310,240), (330, 240), (0,0,0), 1)
                cv2.line(circled_image, (320,230), (320, 250), (0,0,0), 1)
                #cv2.imshow("output", circled_image)
                #cv2.waitKey(0)
                
                image_count+=1
                cv2.imwrite("ed_see_"+str(image_count)+".jpg",circled_image)
                key = cv2.waitKey(1) & 0xFF
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)
                # press the 'q' key to stop the video stream
                if key == ord("p"):
                    #print(sum_1)
                    break
                
                return x,y, pos,angle
                
                
 
def get_angle(x,y):
    diff = x - 320
    if np.abs(diff)<14 :
        print("I am aligned!")
        direction = None
    elif diff < 0:
        direction = 'a'
    else:
        direction = 's'
    
    angle = np.abs(diff) * 0.07

    return angle, direction

def self_orient(pos,angle):
    #scan environment for target object    
    x,y, pos, angle = get_target(pos, angle)
    print("Current vehicle orientation(line 137): ",angle)
      
    #once found (hull is not empty), stop moving, get angle     
    degree, direction = get_angle(x,y)
    print("ANGLE AND DIRECTION: ", degree, direction)
    
    #turn for said angle
    if direction is None:
        return pos, angle
    if direction == 'a':        
        pos, angle = left(degree, pos, angle)
        print("Current vehicle orientation(line 148): ",angle)
    if direction == 's':        
        pos, angle = right(degree, pos, angle)
        print("Current vehicle orientation(line 152): ",angle)
    return pos, angle
    
def distance():
    gpio.setmode(gpio.BOARD)
    gpio.setup(trig,gpio.OUT)
    gpio.setup(echo,gpio.IN)

    #Ensure output has no value
    gpio.output(trig, False)
    time.sleep(0.01)

    #Generate trigger pulse
    gpio.output(trig, True)
    time.sleep(0.00001)
    gpio.output(trig, False)

    #Generate echo time signal
    while gpio.input(echo) == 0:
        pulse_start = time.time()
    while gpio.input(echo) == 1:
        pulse_end = time.time()
        
    try:
        pulse_duration = pulse_end - pulse_start
    except:
        distance = 0
        gpio.cleanup()
        return distance
        
    #Convert time to distance
    distance = (pulse_duration*17150)/100
    distance = round(distance, 2)

    #Cleanup gpio pins and return distance estimate
    gpio.cleanup()
    return distance


    
def imu_check(angle):
    #identify serial connection

    while True:
        if(ser.in_waiting >0):
            
            #read serial stream
            line = ser.readline()
            line = line.rstrip().lstrip()
            line=str(line)
            line = line.strip("'")
            line = line.strip("b'")
            line = line[:-2]
            current_ori = float(line)
            current_ori = 360 - current_ori
            
            if angle>180:
                angle = angle-360
            if current_ori>180:
                current_ori = current_ori - 360
            
            ori_low = angle-1.5
            ori_up = angle+1.5
            
            if current_ori >= ori_low and current_ori <= ori_up :
                print("Current IMU ori: ", current_ori)
                return True
            
            
def get_imu_pos():
    while True:
        if(ser.in_waiting >0):
            
            #read serial stream
            line = ser.readline()
            line = line.rstrip().lstrip()
            line=str(line)
            line = line.strip("'")
            line = line.strip("b'")
            line = line[:-2]
            current_ori = float(line)
            
            current_ori = 360 - current_ori
            
            if current_ori>180:
                current_ori = current_ori - 360
            
            print("Current IMU ori(line 256): ", current_ori)
            return current_ori
                


def calculate_pos(pos, angle, value, degree):
    angle += degree
    x,y = pos[0], pos[1]
    pos = (x+round(np.cos(np.deg2rad(angle))*value, 2), y+round(np.sin(np.deg2rad(angle))*value, 2))
    #pos[0] += round(np.cos(np.deg2rad(angle))*value, 2)
    #pos[1] += round(np.sin(np.deg2rad(angle))*value, 2)
    if np.abs(angle)>=360:
        angle = angle-360
    f.write(str(pos)+" "+str(angle)+"\n")   
    print("Current vehicle orientation(line 280): ",angle)
    print("Current position and orientation: ", pos, angle)
    return pos, angle
    
    
        
def calculate_ticks(direction,value):  
    ticks = 0

    if direction == "w" :
        ticks = int(round(98*value,2))
    
    elif direction == "z":
        ticks = int(round(98*value,2))
        value = -value   
    return ticks
        
        
        
def move(pwm1, pwm2, ticks):
    counterBR= np.uint64(0)
    counterFL=np.uint64(0)
    buttonBR = int(0)
    buttonFL = int(0)
    
    while True:
        #print("counterBR =", counterBR, "counterFL =", counterFL, "BR state: ", gpio.input(12), "FL state: ", gpio.input(7))
                
        if int(gpio.input(12)) != int(buttonBR):
            buttonBR = int(gpio.input(12))
            counterBR += 1
        if int(gpio.input(7)) != int(buttonFL):
                buttonFL = int(gpio.input(7))
                counterFL += 1        
            #print(counter)
                            
        if counterFL>ticks and counterBR >ticks:
            pwm1.stop()
            pwm2.stop()
            gameover()
            break
    
    return 

    

def forward(distance, pos, angle):
    pos, angle = calculate_pos(pos, angle, distance, 0)
    init()
    ticks = calculate_ticks('w',distance)
    pwm1 = gpio.PWM(31,50)
    pwm2 = gpio.PWM(37,50)
    val = 50
    pwm1.start(val)
    pwm2.start(val)
    #wait
    time.sleep(0.1)
    move(pwm1,pwm2,ticks)
    return pos, angle
   


def reverse(distance, pos, angle):
    pos, angle = calculate_pos(pos, angle, distance, 0)
    init()
    ticks = calculate_ticks('z',distance)
    pwm1 = gpio.PWM(33,50)
    pwm2 = gpio.PWM(35,50)
    val = 50
    pwm1.start(val)
    pwm2.start(val)
    #wait
    time.sleep(0.1)
    move(pwm1,pwm2,ticks)
    return pos, angle


def left(degree, pos, angle):
    pos, angle = calculate_pos(pos, angle, 0, degree)
    init()
    pwm1 = gpio.PWM(33,50)
    pwm2 = gpio.PWM(37,50)
    #val = 40 #bench
    val = 40
    pwm1.start(val)
    pwm2.start(val)
    #wait
    time.sleep(0.1)
    
    if (imu_check(angle)):
        pwm1.stop()
        pwm2.stop()
        gameover()
    
    return pos, angle


def right(degree, pos, angle):
    pos, angle = calculate_pos(pos, angle, 0, -degree)
    init()
    pwm1 = gpio.PWM(31,50)
    pwm2 = gpio.PWM(35,50)
    #val = 40
    val = 40
    pwm1.start(val)
    pwm2.start(val)
    #wait
    time.sleep(0.1)
    
    if (imu_check(angle)):
        pwm1.stop()
        pwm2.stop()
        gameover()
    
    return pos, angle

def open_gripper():
    init()
    pwm.ChangeDutyCycle(9.5)
    time.sleep(1)
    gpio.cleanup()
    
def close_gripper():
    init()
    pwm.ChangeDutyCycle(5.5)
    time.sleep(1)
    gpio.cleanup()

print("Hi, please wait for initializaton routine to complete.")
init()

print("STARTING ORIENTATION : ", angle)
print("Stating IMU orientation: ",get_imu_pos())


choice = input("Enter color choice (r or b): ")

# lower_mask = np.array([50, 0, 10])
# upper_mask = np.array([80, 255, 255])

if choice.lower() == 'b':
    lower_mask = np.array([28, 42, 93])
    upper_mask = np.array([130, 255, 255])
elif choice.lower() == 'r':
    lower_mask = np.array([116, 49, 111])
    upper_mask = np.array([199, 255, 255])
else:
    lower_mask = np.array([0, 0, 25])
    upper_mask = np.array([192, 88, 63])

while True:
    ####PIPELINE####
    #scan environment for target object    
    x,y, pos, angle = get_target(pos,angle)
    print("X and Y: ", x, y)
    print("Current position and orientation: ", pos, angle)
      
    #once found (hull is not empty), stop moving, get angle     
    degree, direction = get_angle(x,y)
    print("ANGLE AND DIRECTION: ", degree, direction)
    print("Current position and orientation: ", pos, angle)
    
    
    #turn for said angle
    if direction is None:
        print("I am done with first loop")
        print("Current position and orientation: ", pos, angle)
        #break
    
    elif direction == 'a':
        pos, angle = left(degree, pos, angle)
        print("Current vehicle orientation: ",angle)
        print("Current position and orientation: ", pos, angle)
    
    elif direction == 's':
        pos, angle = right(degree, pos, angle)
        print("Current vehicle orientation: ",angle)
        print("Current position and orientation: ", pos, angle)
        
    
    #get dist - if dist reading is zero (target might not be close enough) - repeat pipeline
    initial_dist = distance()
    print("Ultrasonic initial reading: ", initial_dist)
   
    if initial_dist == 0 :
        print("Did not get distance reading, moving forward")
        pos, angle = forward(0.1, pos, angle)
        print("Current vehicle orientation: ",angle)
        print("Current position and orientation: ", pos, angle)
        continue
    
    else:
        print("I am done with first loop")
        break
        
 
distance_to_cover = initial_dist/4
counter = 0
 
if initial_dist>0.11:
    counter +=1
    print("I am in the second loop")
    pos, angle = forward(distance_to_cover, pos, angle)
    print("Covering distance segment number ", counter)
    print("Distance to cover: ", distance_to_cover)
    current_distance = distance()
    while current_distance > 0.11:
        print("Current vehicle orientation: ",angle)
        print("Going into self_orient")
        print("CUrrent distance: ", current_distance)
        pos, angle = self_orient(pos,angle)
        distance_to_cover = current_distance/4
        if distance_to_cover <=0.2:
            open_gripper()
        pos, angle = forward(distance_to_cover, pos, angle)
        print("Current vehicle orientation: ",angle)
        current_distance = distance() 
    
    
#open_gripper()
close_gripper()
#gameover()
#d = distance()
#print("Distance after picking up obstacle: ", d)
#pos, angle = left(45, pos, angle)
#d1 = distance()
#print("Distance after picking up obstacle: ", d1)
#if np.abs(d-d1)<=5:
#    print("I HAVE THE TARGET!!!!!!!!!!!")

#Sequence of actions to move to the dropoff location and leave the target##
print("Current position and orientation: ", pos, angle)

#Turn to orient with 0 degrees
angle_to_zero = 0 - angle
if angle_to_zero == 0:
    print("No need to go to zero, already at zero")
elif angle_to_zero<0:
    #angle must have been positive (facing left) so turn right
    print("to go to zero, turn right by ",np.abs(angle_to_zero))
    pos, angle = right(np.abs(angle_to_zero), pos, angle)
elif angle_to_zero>0:
    #angle must have been negative (facing right) so turn left
    print("to go to zero, turn left by ",np.abs(angle_to_zero))
    pos, angle = left(np.abs(angle_to_zero), pos, angle)
    
print("Current position and orientation: ", pos, angle)   
#calculate angle to turn by to face dropoff location and distance to move by
x_dropoff, y_dropoff = 0,0
angle_to_dropoff = np.rad2deg(math.atan2(y_dropoff-pos[1],x_dropoff-pos[0]))
print("Angle to dropoff has been calculated as : ", angle_to_dropoff)
dist_to_dropoff = math.hypot(x_dropoff-pos[0], y_dropoff-pos[1])
print("Distance to dropoff has been calculated as : ", dist_to_dropoff)

#provide suitable actions.
if angle_to_dropoff<0:
    print("Turning right")
    pos, angle = right(np.abs(angle_to_dropoff), pos, angle)
    print("Current position and orientation: ", pos, angle)   
elif angle_to_dropoff>0:
    print("Turning left")
    pos, angle = left(np.abs(angle_to_dropoff), pos, angle)
    print("Current position and orientation: ", pos, angle)   

print("Moving forward")
pos, angle = forward(dist_to_dropoff, pos, angle)
print("Current position and orientation: ", pos, angle)   

print("Reached drop off location")
print("Current position and orientation: ", pos, angle)

open_gripper()
pos, angle = reverse(0.1, pos, angle)
close_gripper()
    
#Turn to orient with 0 degrees
angle_to_zero = 0 - angle
if angle_to_zero == 0:
    print("No need to go to zero, already at zero")
elif angle_to_zero<0:
    #angle must have been positive (facing left) so turn right
    print("to go to zero, turn right by ",np.abs(angle_to_zero))
    pos, angle = right(np.abs(angle_to_zero), pos, angle)
elif angle_to_zero>0:
    #angle must have been negative (facing right) so turn left
    print("to go to zero, turn left by ",np.abs(angle_to_zero))
    pos, angle = left(np.abs(angle_to_zero), pos, angle)
    
print("Current position and orientation: ", pos, angle)
"""
#calculate angle to turn by to face dropoff location and distance to move by
x_dropoff, y_dropoff = 0,0
angle_to_dropoff = np.rad2deg(math.atan2(y_dropoff-pos[1],x_dropoff-pos[0]))
print("Angle to dropoff has been calculated as : ", angle_to_dropoff)
dist_to_dropoff = math.hypot(x_dropoff-pos[0], y_dropoff-pos[1])
print("Distance to dropoff has been calculated as : ", dist_to_dropoff)

#provide suitable actions.
if angle_to_dropoff<0:
    print("Turning right")
    pos, angle = right(np.abs(angle_to_dropoff), pos, angle)
    print("Current position and orientation: ", pos, angle)   
elif angle_to_dropoff>0:
    print("Turning left")
    pos, angle = left(np.abs(angle_to_dropoff), pos, angle)
    print("Current position and orientation: ", pos, angle)   

print("Moving forward")
pos, angle = forward(dist_to_dropoff, pos, angle)
print("Current position and orientation: ", pos, angle)   

print("Reached drop off location")
print("Current position and orientation: ", pos, angle)
"""

f.close()
    


















