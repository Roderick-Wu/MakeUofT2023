'''
Arm THing
'''

import time
import board
import pwmio
import math
import digitalio
from adafruit_motor import servo
# create a PWMOut object on Pin A2.

armZenithPWM1 = pwmio.PWMOut(board.GP15, duty_cycle = 2 ** 15, frequency=50) #Left
armZenithPWM2 = pwmio.PWMOut(board.GP16, duty_cycle = 2 ** 15, frequency=50) #Right
armAzimuthPWM = pwmio.PWMOut(board.GP17, duty_cycle = 2 ** 15, frequency=50)
elbowPWM = pwmio.PWMOut(board.GP18, duty_cycle = 2 ** 15, frequency=50)
wristPWM = pwmio.PWMOut(board.GP19, duty_cycle = 2 ** 15, frequency=50)
# Create a servo object, my_servo.
armZenith1 = servo.Servo(armZenithPWM1)
armZenith2 = servo.Servo(armZenithPWM2)
armAzimuth = servo.Servo(armAzimuthPWM)
elbow = servo.Servo(elbowPWM)
wrist = servo.Servo(wristPWM)

restIndicator = digitalio.DigitalInOut(board.GP12)
restIndicator.direction = digitalio.Direction.OUTPUT
movingIndicator = digitalio.DigitalInOut(board.GP13)
movingIndicator.direction = digitalio.Direction.OUTPUT

receiver = digitalio.DigitalInOut(board.GP11)
receiver.direction = digitalio.Direction.INPUT
receiver.pull = digitalio.Pull.DOWN

def getAngles(x, y):
    d = (x**2 + y**2)**(1/2)
    theta = (math.acos(d/2))*180/(math.pi)
    alpha = 180 - 2*theta
    beta = theta + 90
    phi = math.asin(y/d)*180/(math.pi)
    return phi, theta, alpha, beta

def moveTo(curCoords, goTo):
    newCoords = curCoords[:]
    for i in range(4):
        if curCoords[i] < goTo[i]:
            newCoords[i] += 1
        elif curCoords[i] > goTo[i]:
            newCoords[i] -= 1
    return newCoords

angles = [90, 90, 90, 0]  #Azimuth, Zenith, Elbow, Wrist
elbowT = 40

armAzimuth.angle = angles[0]
armZenith1.angle = angles[1]
armZenith2.angle = 180 - angles[1]
elbow.angle = 180 - angles[2] + elbowT
wrist.angle = 180 - angles[3]

restIndicator.value = True
movingIndicator.value = False
getMoving = False

time.sleep(5)

xin = 0
yin = 1

while True:
    print("resting")
    
    #================
    
    while restIndicator.value:
        record = False
        
        while not record:
            if receiver.value:
                record = True
                break
            print("waiting signal")
            xin = 0
            yin = 0
            time.sleep(0.05)
        
        while record:
            xin += 0.05
            time.sleep(0.05)
            print("x")
            
            if not receiver.value:
                record = False
                break
                
        while not record:
            if receiver.value:
                record = True
                break
            print("waiting signal 2")
            time.sleep(0.05)
        
        while record:
            yin += 0.05
            time.sleep(0.05)
            print("y")
            
            if not receiver.value:
                record = False
                restIndicator.value = False
                movingIndicator.value = True
                getMoving = True
                
                if xin > 1.2:
                    xin = 1.2
                if yin > 1.2:
                    yin = 1.2
                print(xin, yin)
                
                p, t, a, b = getAngles(xin, 0.4 + yin)
                p = round(p)
                t = round(t)
                a = round(a)
                b = round(b)
                print (p, t, a, b)
                break
        
        if getMoving:
            break
    #===================

    while getMoving:
        armAzimuth.angle = angles[0]
        armZenith1.angle = angles[1]
        armZenith2.angle = 180 - angles[1]
        elbow.angle = 180 - angles[2] + elbowT
        wrist.angle = 180 - angles[3]
        
        #=========================================
        if armAzimuth.angle < 0:
            armAzimuth.angle = 0
        elif armAzimuth.angle > 180:
            armAzimuth.angle = 180
            
        if armZenith1.angle < 0:
            armZenith1.angle = 0
        elif armZenith1.angle > 180:
            armZenith1.angle = 180
            
        if armZenith2.angle < 0:
            armZenith2.angle = 0
        elif armZenith2.angle > 180:
            armZenith2.angle = 180
            
        if elbow.angle < 0:
            elbow.angle = 0
        elif elbow.angle > 180:
            elbow.angle = 180
            
        if wrist.angle < 0:
            wrist.angle = 0
        elif wrist.angle > 180:
            wrist.angle = 180
        #=================================

        ptab = moveTo(angles, (p, t, a, b))
        angles = [ptab[0], ptab[1], ptab[2], ptab[3]]
        
        restIndicator.value = False
        movingIndicator.value = True
        
        if angles[0] == p and angles[1] == t and angles[2] == a and angles[3] == b:
            getMoving = False
            restIndicator.value = True
            movingIndicator.value = False
            
            break
        
        print("moving")
        time.sleep(0.01)

    time.sleep(0.01)







    '''
    for angle in range(0, 180, 1):
        armZenith1.angle = angle
        armZenith2.angle = 180 - angle
        armAzimuth.angle = angle
        elbow.angle = angle
        time.sleep(0.001)

    for angle in range(180, 0, -1):
        armZenith1.angle = angle
        armZenith2.angle = 180 - angle
        armAzimuth.angle = angle
        elbow.angle = angle
        time.sleep(0.001)
    '''
    '''
    # 0 - 180 degrees, 5 degrees at a time.
    print('Rotating to 180')
    for angle in range(0, 180, 1):
        my_servo.angle = angle
        time.sleep(0.01)
    # 180 - 0 degrees, 5 degrees at a time.
    print('Rotating to 0')
    for angle in range(180, 0, -1):
        my_servo.angle = angle
        time.sleep(0.01)
    '''
