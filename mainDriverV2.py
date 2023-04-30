import RPi.GPIO as GPIO
import threading
import math
import time

GPIO.cleanup()
motorPins2 = (31, 33, 35, 37)
motorPins1 = (12, 16, 18, 22)    # define pins connected to four phase ABCD of stepper motor
CCWStep = (0x01,0x02,0x04,0x08) # define power supply order for rotating anticlockwise
CWStep = (0x08,0x04,0x02,0x01)  # define power supply order for rotating clockwise

btns = [7,11,13,15]
leds = [36,38,40]

def setup():
    GPIO.setmode(GPIO.BOARD)       # use PHYSICAL GPIO Numbering
    for pin in motorPins1:
        GPIO.setup(pin,GPIO.OUT)
    for pin in motorPins2:
        GPIO.setup(pin, GPIO.OUT)
    for btn in btns:
        GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    for led in leds:
        GPIO.setup(led, GPIO.OUT)
        GPIO.output(led, GPIO.LOW)

# as for four phase stepping motor, four steps is a cycle. the function is used to drive the stepping motor clockwise or anticlockwise to take four steps
def moveOnePeriod1(direction,ms):
    for j in range(0,4,1):      # cycle for power supply order
        for i in range(0,4,1):  # assign to each pin
            if (direction == 1):# power supply order clockwise
                GPIO.output(motorPins1[i],((CCWStep[j] == 1<<i) and GPIO.HIGH or GPIO.LOW))
            else :              # power supply order anticlockwise
                GPIO.output(motorPins1[i],((CWStep[j] == 1<<i) and GPIO.HIGH or GPIO.LOW))
        if(ms<3):       # the delay can not be less than 3ms, otherwise it will exceed speed limit of the motor
            ms = 3
        time.sleep(ms*0.001)

def moveOnePeriod2(direction,ms):
    for j in range(0,4,1):      # cycle for power supply order
        for i in range(0,4,1):  # assign to each pin
            if (direction == 1):# power supply order clockwise
                GPIO.output(motorPins2[i],((CCWStep[j] == 1<<i) and GPIO.HIGH or GPIO.LOW))
            else :              # power supply order anticlockwise
                GPIO.output(motorPins2[i],((CWStep[j] == 1<<i) and GPIO.HIGH or GPIO.LOW))
        if(ms<3):       # the delay can not be less than 3ms, otherwise it will exceed speed limit of the motor
            ms = 3
        time.sleep(ms*0.001)

# continuous rotation function, the parameter steps specifies the rotation cycles, every four steps is a cycle
def moveSteps1(direction, ms, steps):
    for i in range(steps):
        moveOnePeriod1(direction, ms)

def moveSteps2(direction, ms, steps):
    for i in range(steps):
        moveOnePeriod2(direction, ms)
# function used to stop motor
def motorStop1():
    for i in range(0,4,1):
        GPIO.output(motorPins1[i],GPIO.LOW)

def motorStop2():
    for i in range(0,4,1):
        GPIO.output(motorPins2[i],GPIO.LOW)

def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

###################################################################################

#Conversion: 512 steps = 41 mm NEW: 1000steps : 90mm
#Canvas size: 170mm x 270mm
#Motor to motor distance: 580mm
#L.R.Motor to default pen position distance: 590mm,590mm
#Top left offset from 0x0: 200x145
canvas_size = (170, 270)
top_left_offset = (200,200)
step_mm_conversion = (90, 1000)
distance_between_motors = 580

lllr = [6555, 6555]
#imgs = [[195,170]]
imgs = []
f = open("output_tracepath.txt", "r")
sample_array = []
for line in f.readlines():
    temp = line.split(" ")
    temp[1] = temp[1].strip("\n")
    sample_array.append([temp[0], temp[1]])
f.close()

for arr in sample_array:
    arr = [int(numeric_string) for numeric_string in arr]
    imgs.append(arr)

imgs = [[0,0],[10,0],[10,10],[0,10],
        [0,1],[9,1],[9,9],[1,9],
        [1,2],[8,2],[8,8],[2,8],
        [2,3],[7,3],[7,7],[3,7],
        [3,4],[6,4],[6,6],[4,6]]
"""
imgs = [[0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0],
        [0,0],[1,0],[1,1],[0,1],[0,0]]
"""
# top left corner offset = 185x170 [195,170]

def calculate(img, max_x, min_y):
    global lllr
    
    dirLl, dirLr = None, None
    speedLl, speedLr = None, None
    distLl, distLr = None, None
    
    remapX = remap(img[0], 0, max_x, 0, canvas_size[0])
    remapY = remap(img[1], 0, min_y, 0, canvas_size[1])
    
    targetX = remapX + top_left_offset[0] # - xy_offset_overtime_current
    targetY = remapY + top_left_offset[1] # - xy_offset_overtime_current
    
    newLl = (math.sqrt((targetX)**2+targetY**2)/step_mm_conversion[0]*step_mm_conversion[1])
    newLr = (math.sqrt((distance_between_motors-targetX)**2+targetY**2)/step_mm_conversion[0]*step_mm_conversion[1])
    
    
    distLl = int(lllr[0]-newLl)
    distLr = int(lllr[1]-newLr)
    print(lllr, (targetX,targetY), (newLl, newLr), (distLl, distLr))
    
    lllr[0] = newLl
    lllr[1] = newLr
    
    if distLl >= 0:
        dirLl = 1
    elif distLl <= 0:
        dirLl = 0
        distLl *= -1
    if distLr >= 0:
        dirLr = 0
    elif distLr <=0:
        dirLr = 1
        distLr *= -1
    
    if distLl == 0 or distLr == 0:
        speedLl = 3
        speedLr = 3
    else:
        if distLl >= distLr:
            speedLl = 3
            speedLr = 3*distLl/distLr
        elif distLl <= distLr:
            speedLr = 3
            speedLl = 3*distLr/distLl
        
        if speedLl <= 0:
            speedLl *= -1
        if speedLr <= 0:
            speedLr *= -1
    
    directionLlLr = [dirLl, dirLr]
    speedLlLr = [speedLl, speedLr]
    distanceLlLr = [distLl, distLr]
    
    return directionLlLr, speedLlLr, distanceLlLr
   


###################################################################################
def destroy():
    GPIO.cleanup()             # Release resource

if __name__ == '__main__':     # Program entrance
    print ('Program is starting...')
    start_time = time.time()
    setup()
    max_x = max(row[0] for row in imgs)
    min_y = max(row[1] for row in imgs)
    
    index = 0
    xy_offset_overtime_increment = 10/7000
    xy_offset_overtime_max = xy_offset_overtime_increment*len(imgs)
    xy_offset_overtime_current = 0
    pen_setup = True
    while pen_setup:
        if not(GPIO.input(btns[1])) and not(GPIO.input(btns[2])):
            m1 = [1, 3, 5]
            th1 = threading.Thread(target=moveSteps1, args=(m1[0],m1[1],m1[2],))
            th1.start()
            th1.join()
            GPIO.output(leds[0], GPIO.HIGH)
        elif not(GPIO.input(btns[1])) and not(GPIO.input(btns[3])):
            m2 = [0, 3, 5]
            th2 = threading.Thread(target=moveSteps2, args=(m2[0],m2[1],m2[2],))
            th2.start()
            th2.join()
            GPIO.output(leds[0], GPIO.HIGH)
        else:
            if not(GPIO.input(btns[2])):
                m1 = [0, 3, 5]
                th1 = threading.Thread(target=moveSteps1, args=(m1[0],m1[1],m1[2],))
                th1.start()
                th1.join()
                GPIO.output(leds[0], GPIO.HIGH)
            if not(GPIO.input(btns[3])):
                m2 = [1, 3, 5]
                th2 = threading.Thread(target=moveSteps2, args=(m2[0],m2[1],m2[2],))
                th2.start()
                th2.join()
                GPIO.output(leds[0], GPIO.HIGH)
            if not(GPIO.input(btns[0])):
                pen_setup = False
                GPIO.output(leds[2], GPIO.HIGH)
        time.sleep(0.05)

    for img in imgs:
        GPIO.output(leds[2], GPIO.HIGH)
        GPIO.output(leds[0], GPIO.LOW)
        GPIO.output(leds[1], GPIO.LOW)
        xy_offset_overtime_current += xy_offset_overtime_increment
        index += 1
        print("Vertex", index, "/", str(len(imgs)))
        while True:
            try:
                values = calculate(img, max_x, min_y)
                m1 = [values[0][0], values[1][0], values[2][0]]
                m2 = [values[0][1], values[1][1], values[2][1]]
                th1 = threading.Thread(target=moveSteps1, args=(m1[0],m1[1],m1[2],))
                th2 = threading.Thread(target=moveSteps2, args=(m2[0],m2[1],m2[2],))
                th1.start()
                th2.start()
                th1.join()
                th2.join()
                break
            except KeyboardInterrupt:
                print("Time ellapsed:", time.time()-start_time, "seconds")
                GPIO.output(leds[0], GPIO.HIGH)
                GPIO.output(leds[1], GPIO.LOW)
                GPIO.output(leds[2], GPIO.LOW)
                input("Press <enter> to continue...")
    finish_time = time.time()-start_time
    print("Finished in:", finish_time, "seconds", "or", finish_time/60, "minutes")
    GPIO.output(leds[0], GPIO.HIGH)
    GPIO.output(leds[1], GPIO.HIGH)
    GPIO.output(leds[2], GPIO.HIGH)
