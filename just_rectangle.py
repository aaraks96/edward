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
print("Initializing Pi Camera......")
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


print("Initializing Serial Data......")
ser = serial.Serial('/dev/ttyUSB0',9600)
for i in range(15):
    line = ser.readline()
    
#initialize gpio pins to control servo motor
print("Initializing gpio pins......")
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
    

def get_target(pic):
      
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the current frame
        image = frame.array
        # show the frame to our screen
        #image = cv2.flip(image, -1)
        #cv2.imshow("output", image)
               
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #cv2.imshow("hsv", hsv_image)
        mask = cv2.inRange(hsv_image, lower_mask, upper_mask)
        #cv2.imshow("mask", mask)
        #cv2.waitKey(0)
        _,cnts,h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts)==0:
            rawCapture.truncate(0)
            print("no target found")
            return None, None           
        else:
            rawCapture.truncate(0)
            print("Target obtained")
            cnts_sorted = sorted(cnts, key=cv2.contourArea)
            area = cv2.contourArea(cnts_sorted[-1])
            
            if area <70.0:
                print("Target not big enough")
                return None, None
                
            hull = cv2.convexHull(cnts_sorted[-1])
            (x,y),radius = cv2.minEnclosingCircle(hull)
            print("Circle center", (x,y))
            circled_image = cv2.circle(image,(int(x),int(y)),int(radius),(0,0,255),4)
            cv2.line(circled_image, (310,240), (330, 240), (0,0,0), 1)
            cv2.line(circled_image, (320,230), (320, 250), (0,0,0), 1)
            
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            rawCapture.truncate(0)
            # press the 'q' key to stop the video stream
            if key == ord("p"):
                #print(sum_1)
                break
            
            return x,y

    
def get_pic():
      
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the current frame
        image = frame.array
        # show the frame to our screen
        image = cv2.flip(image, -1)
        #cv2.imshow("output", image)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # press the 'q' key to stop the video stream
        if key == ord("p"):
            #print(sum_1)
            break
        
        return image
                
                
 
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

def self_orient(pic,pos,angle):
    flag = False
    #scan environment for target object    
    x,y = get_target(pic)
    print("Current vehicle orientation(line 137): ",angle)
      
    #once found (hull is not empty), stop moving, get angle     
    if x is not None:
        flag = True
        degree, direction = get_angle(x,y)
        print("ANGLE AND DIRECTION: ", degree, direction)
        
        #turn for said angle
        if direction is None:
            return pos, angle, flag
        if direction == 'a':        
            pos, angle = left(degree, pos, angle)
            print("Current vehicle orientation(line 148): ",angle)
        if direction == 's':        
            pos, angle = right(degree, pos, angle)
            print("Current vehicle orientation(line 152): ",angle)
            
    else:
        return pos, angle, flag
    
    return pos, angle, flag
    
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
    
def calculate_object_location(dist_to_obj, pos, angle):
    x_obj = 0
    y_obj = 0
    x,y = pos[0], pos[1]
    x_obj = x + dist_to_obj*round(np.cos(np.deg2rad(angle))*value, 2)
    y_obj = y + dist_to_obj*round(np.sin(np.deg2rad(angle))*value, 2)
    
    return x_obj, y_obj
    
    
    
    
    
        
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
    val = 50
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
    val = 50
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

print("Please wait for initializaton routine to complete.....")
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

arena_l = 3.00
arena_b = 1.50

######## NEW PIPELINE ##############

pic_count = 0
object_count = 0
dist_to_obj = -1
obstacle_dict = dict()
#1. Create a map of the arena
#start at (0,0) oriented at 0
#turn 45 degrees and take a picture
pos, angle = left(45, pos, angle)  # maneuver 1
pic = get_pic()
cv2.imwrite('image_'+str(pic_count)+'.jpeg', pic)
pic_count+=1
pos, angle, flag = self_orient(pic,pos,angle) #oriented (or not) with target.

if flag == False:
    continue
else:
    object_count+=1
    dist_to_obj = distance()
    x_obj, y_obj = calculate_object_location(dist_to_obj, pos, angle)
    obstacle_dict["obstacle_"+str(object_count)] = {"position": [x_obj, y_obj]}


#go back to orientation 0. 
pos, angle = right(45, pos, angle)  # maneuver 1

#go to next checkpoint 
pos, angle = forward(arena_l/2, pos, angle)  # maneuver 3

pos, angle = left(90, pos, angle)  # maneuver 4
pic = get_pic()
cv2.imwrite('image_'+str(pic_count)+'.jpeg', pic)
pic_count+=1
pos, angle, flag = self_orient(pic,pos,angle) #oriented (or not) with target.

if flag == False:
    continue
else:
    object_count+=1
    dist_to_obj = distance()
    x_obj, y_obj = calculate_object_location(dist_to_obj, pos, angle)
    obstacle_dict["obstacle_"+str(object_count)] = {"position": [x_obj, y_obj]}


#go back to orientation 0. 
pos, angle = right(90, pos, angle)  # maneuver 5


#go to next checkpoint 
pos, angle = forward(arena_l/2, pos, angle)  # maneuver 6

pos, angle = left(135, pos, angle)  # maneuver 7
pic = get_pic()
cv2.imwrite('image_'+str(pic_count)+'.jpeg', pic)
pic_count+=1
pos, angle, flag = self_orient(pic,pos,angle) #oriented (or not) with target.

if flag == False:
    continue
else:
    object_count+=1
    dist_to_obj = distance()
    x_obj, y_obj = calculate_object_location(dist_to_obj, pos, angle)
    obstacle_dict["obstacle_"+str(object_count)] = {"position": [x_obj, y_obj]}


#go back to orientation 0. 
pos, angle = right(45, pos, angle)  # maneuver 8

#go to next checkpoint 
pos, angle = forward(arena_b/2, pos, angle)  # maneuver 9

pos, angle = left(90, pos, angle)  # maneuver 10
pic = get_pic()
cv2.imwrite('image_'+str(pic_count)+'.jpeg', pic)
pic_count+=1
pos, angle, flag = self_orient(pic,pos,angle) #oriented (or not) with target.

if flag == False:
    continue
else:
    object_count+=1
    dist_to_obj = distance()
    x_obj, y_obj = calculate_object_location(dist_to_obj, pos, angle)
    obstacle_dict["obstacle_"+str(object_count)] = {"position": [x_obj, y_obj]}


#go back to orientation 0. 
pos, angle = right(90, pos, angle)  # maneuver 11


#go to next checkpoint 
pos, angle = forward(arena_b/2, pos, angle)  # maneuver 12

pos, angle = left(135, pos, angle)  # maneuver 13
pic = get_pic()
cv2.imwrite('image_'+str(pic_count)+'.jpeg', pic)
pic_count+=1
pos, angle, flag = self_orient(pic,pos,angle) #oriented (or not) with target.

if flag == False:
    continue
else:
    object_count+=1
    dist_to_obj = distance()
    x_obj, y_obj = calculate_object_location(dist_to_obj, pos, angle)
    obstacle_dict["obstacle_"+str(object_count)] = {"position": [x_obj, y_obj]}


#go back to orientation 0. 
pos, angle = right(45, pos, angle)  # maneuver 14

#go to next checkpoint 
pos, angle = forward(arena_l/2, pos, angle)  # maneuver 15

pos, angle = left(90, pos, angle)  # maneuver 16
pic = get_pic()
cv2.imwrite('image_'+str(pic_count)+'.jpeg', pic)
pic_count+=1
pos, angle, flag = self_orient(pic,pos,angle) #oriented (or not) with target.

if flag == False:
    continue
else:
    object_count+=1
    dist_to_obj = distance()
    x_obj, y_obj = calculate_object_location(dist_to_obj, pos, angle)
    obstacle_dict["obstacle_"+str(object_count)] = {"position": [x_obj, y_obj]}


#go back to orientation 0. 
pos, angle = right(90, pos, angle)  # maneuver 17


#go to next checkpoint 
pos, angle = forward(arena_l/2, pos, angle)  # maneuver 18

pos, angle = left(135, pos, angle)  # maneuver 19
pic = get_pic()
cv2.imwrite('image_'+str(pic_count)+'.jpeg', pic)
pic_count+=1
pos, angle, flag = self_orient(pic,pos,angle) #oriented (or not) with target.

if flag == False:
    continue
else:
    object_count+=1
    dist_to_obj = distance()
    x_obj, y_obj = calculate_object_location(dist_to_obj, pos, angle)
    obstacle_dict["obstacle_"+str(object_count)] = {"position": [x_obj, y_obj]}


#go back to orientation 0. 
pos, angle = right(45, pos, angle)  # maneuver 20

#go to next checkpoint 
pos, angle = forward(arena_b/2, pos, angle)  # maneuver 21

pos, angle = left(90, pos, angle)  # maneuver 22
pic = get_pic()
cv2.imwrite('image_'+str(pic_count)+'.jpeg', pic)
pic_count+=1
pos, angle, flag = self_orient(pic,pos,angle) #oriented (or not) with target.

if flag == False:
    continue
else:
    object_count+=1
    dist_to_obj = distance()
    x_obj, y_obj = calculate_object_location(dist_to_obj, pos, angle)
    obstacle_dict["obstacle_"+str(object_count)] = {"position": [x_obj, y_obj]}


#go back to orientation 0. 
pos, angle = right(90, pos, angle)  # maneuver 23

#go to next checkpoint 
pos, angle = forward(arena_b/2, pos, angle)  # maneuver 24

print(obstacle_dict)

print("END OF MAPPING....")


#2. Assign object locations and boundaries #get distance, calculate coordinates

#Using stored values, make a map with boundaries included. These points cannot be occupied by the robot.


#3. Obtain trajectory to the object

#4. Actuate : follow trajectory to object and grab items

#5. Generate  trajectory to dropoff

#6. Actuate: Follow trajectory to dropoff and drop object

#7. Loop to step 1 and update the map











"""

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


f.close()
""" 


















