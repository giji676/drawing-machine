import RPi.GPIO as GPIO
import threading
import math
import time

motor_pins2 = (31, 33, 35, 37)
motor_pins1 = (12, 16, 18, 22)
CCW_step = (0x01, 0x02, 0x04, 0x08)
CW_step = (0x08, 0x04, 0x02, 0x01)
btns = [7, 11, 13, 15]
leds = [36, 38, 40]

GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor_pins1 + motor_pins2, GPIO.OUT)
GPIO.setup(btns, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(leds, GPIO.OUT, initial=GPIO.LOW)


def moveOnePeriod(pins, direction, ms):
    for j in range(0, 4):
        for i, pin in enumerate(pins):
            if direction == 1:
                GPIO.output(pin, CCW_step[j] == 1 << i)
            else:
                GPIO.output(pin, CW_step[j] == 1 << i)
        time.sleep(ms / 1000)


def moveSteps(pins, direction, ms, steps):
    for _ in range(steps):
        moveOnePeriod(pins, direction, ms)


def motorStop(pins):
    for pin in pins:
        GPIO.output(pin, GPIO.LOW)


def remap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


###################################################################################

#Conversion: 512 steps = 41 mm NEW: 1000steps : 90mm
#Canvas size: 170mm x 270mm
#Motor to motor distance: 580mm
#L.R.Motor to default pen position distance: 590mm,590mm
#Top left offset from 0x0: 200x145
canvas_size = (170, 270)
top_left_offset = (200,170)
step_mm_conversion = (90, 1000)
distance_between_motors = 570

lllr = [6555, 6555]
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
    
# top left corner offset = 185x170 [195,170]

def calculate(img, max_x, min_y, debug):
    global lllr
    
    dirLl, dirLr = None, None
    speedLl, speedLr = 3, 3
    distLl, distLr = None, None
    
    remapX = remap(img[0], 0, max_x, 0, canvas_size[0])
    remapY = remap(img[1], 0, min_y, 0, canvas_size[1])
    
    targetX = remapX + top_left_offset[0] # - xy_offset_overtime_current
    targetY = remapY + top_left_offset[1] # - xy_offset_overtime_current
    
    newLl = (math.sqrt((targetX)**2+targetY**2)/step_mm_conversion[0]*step_mm_conversion[1])
    newLr = (math.sqrt((distance_between_motors-targetX)**2+targetY**2)/step_mm_conversion[0]*step_mm_conversion[1])
    
    
    distLl = int(lllr[0]-newLl)
    distLr = int(lllr[1]-newLr)
    if debug:
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

def estimateTime():
    global lllr
    time = 0
    for img in imgs:
        values = calculate(img, max_x, min_y, False)
        m1 = [values[0][0], values[1][0], values[2][0]]
        m2 = [values[0][1], values[1][1], values[2][1]]
        t_m1 = 0
        t_m2 = 0
        for i in range(m1[2]):
            for j in range(0,4,1):
                t_m1 += m1[1]*0.001
        for i in range(m2[2]):
            for j in range(0,4,1):
                t_m2 += m2[1]*0.001
        if t_m1 > t_m2:
            time += t_m1
        else:
            time += t_m2
    
    lllr = [6555, 6555]
    return time

###################################################################################
def destroy():
    GPIO.cleanup()             # Release resource

if __name__ == '__main__':     # Program entrance
    print ('Program is starting...')
    start_time = time.time()
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
            th1 = threading.Thread(target=moveSteps, args=(motor_pins1,m1[0],m1[1],m1[2],))
            th1.start()
            th1.join()
            GPIO.output(leds[0], GPIO.HIGH)
        elif not(GPIO.input(btns[1])) and not(GPIO.input(btns[3])):
            m2 = [0, 3, 5]
            th2 = threading.Thread(target=moveSteps, args=(motor_pins2,m2[0],m2[1],m2[2],))
            th2.start()
            th2.join()
            GPIO.output(leds[0], GPIO.HIGH)
        else:
            if not(GPIO.input(btns[2])):
                m1 = [0, 3, 5]
                th1 = threading.Thread(target=moveSteps, args=(motor_pins1,m1[0],m1[1],m1[2],))
                th1.start()
                th1.join()
                GPIO.output(leds[0], GPIO.HIGH)
            if not(GPIO.input(btns[3])):
                m2 = [1, 3, 5]
                th2 = threading.Thread(target=moveSteps, args=(motor_pins2,m2[0],m2[1],m2[2],))
                th2.start()
                th2.join()
                GPIO.output(leds[0], GPIO.HIGH)
            if not(GPIO.input(btns[0])):
                pen_setup = False
                GPIO.output(leds[2], GPIO.HIGH)
        time.sleep(0.05)

    estimated_time = estimateTime()
    print()
    print("Estimated time:", estimated_time/60, "minutes")
    print()

    for img in imgs:
        GPIO.output(leds[2], GPIO.HIGH)
        GPIO.output(leds[0], GPIO.LOW)
        GPIO.output(leds[1], GPIO.LOW)
        xy_offset_overtime_current += xy_offset_overtime_increment
        index += 1
        print("Vertex", index, "/", str(len(imgs)))
        while True:
            try:
                values = calculate(img, max_x, min_y, True)
                m1 = [values[0][0], values[1][0], values[2][0]]
                m2 = [values[0][1], values[1][1], values[2][1]]

                if (m1[2] == 0):
                    th2 = threading.Thread(target=moveSteps, args=(motor_pins2,m2[0],m2[1],m2[2],))
                    th2.start()
                    th2.join()
                    time.sleep(m2[1] * m2[2])
                    
                elif (m2[2] == 0):
                    th1 = threading.Thread(target=moveSteps, args=(motor_pins1,m2[0],m2[1],m2[2],))
                    th1.start()
                    th1.join()
                    time.sleep(m1[1] * m1[2])
                
                else:
                    th1 = threading.Thread(target=moveSteps, args=(motor_pins1,m2[0],m2[1],m2[2],))
                    th2 = threading.Thread(target=moveSteps, args=(motor_pins2,m2[0],m2[1],m2[2],))
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
