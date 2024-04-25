import RPi.GPIO as GPIO
from time import sleep

motor1 = 12
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor1,GPIO.OUT)
motor1_pwm = GPIO.PWM(motor1,1000)
motor1_pwm.start(0)

D = 0.1

while True:
    motor1_pwm.ChangeDutyCycle(D)