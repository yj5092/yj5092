<<<<<<< HEAD
import threading
import serial
import time
import RPi.GPIO as GPIO

PWMA = 18
AIN1 = 22
AIN2 = 27
PWMB = 23
BIN1 = 24
BIN2 = 25

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)

L_Motor = GPIO.PWM(PWMA, 100)
R_Motor = GPIO.PWM(PWMB, 100)
L_Motor.start(0)  
R_Motor.start(0)

bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)
gData = ""

def set_L_motor(speed): 
    if speed > 0:
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.HIGH)
        L_Motor.ChangeDutyCycle(speed)
    elif speed < 0:
        GPIO.output(AIN1, GPIO.HIGH)
        GPIO.output(AIN2, GPIO.LOW)
        L_Motor.ChangeDutyCycle(abs(speed))
    else:  
        GPIO.output(AIN1, GPIO.LOW)
        GPIO.output(AIN2, GPIO.LOW)
        L_Motor.ChangeDutyCycle(0)

def set_R_motor(speed):  
    if speed > 0:
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.HIGH)
        R_Motor.ChangeDutyCycle(speed)
    elif speed < 0:
        GPIO.output(BIN1, GPIO.HIGH)
        GPIO.output(BIN2, GPIO.LOW)
        R_Motor.ChangeDutyCycle(abs(speed))
    else: 
        GPIO.output(BIN1, GPIO.LOW)
        GPIO.output(BIN2, GPIO.LOW)
        R_Motor.ChangeDutyCycle(0)

def go(speed=100):
    set_L_motor(speed)
    set_R_motor(-speed)
    print("Go")

def back(speed=100):
    set_L_motor(-speed)
    set_R_motor(speed)
    print("Back")

def left(speed=50):
    set_L_motor(-speed)
    set_R_motor(-speed)
    print("Left")

def right(speed=50):
    set_L_motor(speed)
    set_R_motor(speed)
    print("Right")

def stop():
    set_L_motor(0)
    set_R_motor(0)
    print("Stop!")

def serial_thread(): 
    global gData
    while True:
        data = bleSerial.readline()
        data = data.decode()
        gData = data

def main():
    global gData
    try:
        while True:
            if gData.find("B5") >= 0:
                gData = ""
                go()
            
            elif gData.find("B1") >= 0:
                gData = ""
                back()
            
            elif gData.find("B3") >= 0:
                gData = ""
                left() 

            elif gData.find("B2") >= 0:
                gData = ""
                right() 

            elif gData.find("B0") >= 0:
                gData = ""
                stop() 
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    task1 = threading.Thread(target = serial_thread)
    task1.start()
    main()
    bleSerial.close()
    GPIO.cleanup()

=======
import cv2
import numpy as np

image_names = ['1.jpg', '2.jpg', '3.jpg', '4.jpg']

for name in image_names:
    # ① 이미지 읽기
    img = cv2.imread(name)
    img = cv2.resize(img, (640, 480))
    
    # ② 노이즈 제거
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    
    # ③ BGR → HSV 변환
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # ④ 색상 범위 지정 (노란색 / 흰색)
    # ※ 조명 변화 대응을 위해 S, V 범위 좁게 설정
    lower_yellow = np.array([18, 80, 100])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_white = np.array([0, 0, 210])
    upper_white = np.array([180, 40, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    # ⑤ 노란색과 흰색 결합
    mask = cv2.bitwise_or(mask_yellow, mask_white)

    # ⑥ 형태학적 연산으로 잡음 제거 (모폴로지)
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

    # ⑦ 윤곽선 검출
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = img.copy()
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 300:  # 너무 작은 점/노이즈 제외
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # ⑧ 최종 결과 표시 및 저장
    cv2.imshow(name, result)
    cv2.imwrite(f"result_{name}", result)

cv2.waitKey(0)
cv2.destroyAllWindows()
>>>>>>> first commit
