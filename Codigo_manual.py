import machine
from time import sleep

while True:
    try:
        print("Hello World!")
        machine.pinout()
        sleep(1)
    except:
        break