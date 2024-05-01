import RPi.GPIO as GPIO
from time import sleep

motor1_rev= 12
motor1_fwd = 13
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(motor1_fwd,GPIO.OUT)
motor1_fwd_pwm = GPIO.PWM(motor1_fwd,10000)
motor1_fwd_pwm.start(0)

GPIO.setup(motor1_rev,GPIO.OUT)
motor1_rev_pwm = GPIO.PWM(motor1_rev,10000)
motor1_rev_pwm.start(0)

for duty in range(0,101,1):
    motor1_fwd_pwm.ChangeDutyCycle(duty)
    motor1_rev_pwm.ChangeDutyCycle(0)
    sleep(0.01)
sleep(1)

for duty in range(100,-1,-1):
    motor1_fwd_pwm.ChangeDutyCycle(duty)
    motor1_rev_pwm.ChangeDutyCycle(0)
    sleep(0.01)
sleep(2)

for duty in range(0,101,1):
    motor1_fwd_pwm.ChangeDutyCycle(0)
    motor1_rev_pwm.ChangeDutyCycle(duty)
    sleep(0.01)
sleep(1)

for duty in range(100,-1,-1):
    motor1_fwd_pwm.ChangeDutyCycle(0)
    motor1_rev_pwm.ChangeDutyCycle(duty)
    sleep(0.01)