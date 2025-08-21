import machine
from time import sleep

"""Comentario so para atualizar o git"""


while True:
    try:
        print("Hello World!")
        machine.pinout()
        sleep(1)
    except:
        break


# teste
