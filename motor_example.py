from gpiozero import Motor
from time import sleep

from aiy.pins import PIN_A,PIN_B,PIN_C,PIN_D

motor1=Motor(PIN_A,PIN_B)
motor2=Motor(PIN_D,PIN_C)

for x in range (4):
    print(x)
    motor1.forward()
    motor2.forward()
    sleep(2)
    motor1.backward()
    motor2.backward()
    sleep(2)
motor1.stop()
motor2.stop()