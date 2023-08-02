import sys
from threading import Thread
import time
import serial as ser
import RPi.GPIO as gpio
from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

board = Board(1, 0x10)  # RaspberryPi select bus 1, set address to 0x10
fastLight_pin = 33
slowLight_pin = 40
ser = ser.Serial('/dev/ttyS0', 9600)
is_blink = False
is_doorOpended = False




def board_detect():
    l = board.detecte()
    print("Board list conform:")
    print(l)


def print_board_status():
    if board.last_operate_status == board.STA_OK:
        print("board status: everything ok")
    elif board.last_operate_status == board.STA_ERR:
        print("board status: unexpected error")
    elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
        print("board status: device not detected")
    elif board.last_operate_status == board.STA_ERR_PARAMETER:
        print("board status: parameter error, last operate no effective")
    elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
        print("board status: unsupport board framware version")

def open_door():
    for i in range(4):  # slow to fast
        board.motor_movement([board.M2], board.CW, 100)  # DC motor 1 movement, orientation clockwise
        time.sleep(1)
    board.motor_stop(board.ALL)  # stop all DC motor
    print("********************* open door completed *********************")

def close_door():
    for i in range(4):  # fast to low
        board.motor_movement([board.M2], board.CCW, 100)  # DC motor 1 movement, orientation clockwise
        time.sleep(1)
    board.motor_stop(board.ALL)  # stop all DC motor
    print("********************* open door completed *********************")

def light_blink(pin):
    while True:
        if is_blink == False:
            break
        gpio.output(pin, gpio.HIGH)
        time.sleep(0.5)
        gpio.output(pin, gpio.LOW)





if __name__ == "__main__":

    board_detect()  # If you forget address you had set, use this to detected them, must have class instance
    while board.begin() != board.STA_OK:  # Board begin and check board status
        print_board_status()
        print("board begin faild")
        time.sleep(2)
    print("board begin success")
    board.set_encoder_enable(board.ALL)  # Set selected DC motor encoder enable
    board.set_encoder_reduction_ratio(board.ALL, 43)  # Set selected DC motor encoder reduction ratio
    board.set_moter_pwm_frequency(10000)  # Set DC motor pwm frequency to 1000HZ

    gpio.setmode(gpio.BOARD)
    gpio.setwarnings(False)
    gpio.setup([fastLight_pin, slowLight_pin], gpio.OUT)


    #set fast/slow light
    while True:
        received_data = ser.read(1).decode('utf-8')
        # open door: 1, close door: 2, open fast light: 3, open slow light: 4, close fast/slow light: 5,
        if received_data == '1':
            if is_doorOpended == False:
                open_door()
                is_doorOpended = True
                ser.write(b'0') # After opening the door, return signal 0 to MCU
        elif received_data == '2':
            if is_doorOpended == True:
                close_door()
                is_doorOpended = False
                ser.write(b'0') # # After closing the door, return signal 0 to MCU
        elif received_data == '3':
            is_blink = True
            t_openLight = Thread(target=light_blink, args=(fastLight_pin,))
            ser.write(b'0')  # # After closing the door, return signal 0 to MCU
        elif received_data == '4':
            is_blink = True
            t_openLight = Thread(target=light_blink, args=(slowLight_pin,))
            ser.write(b'0')  # # After closing the door, return signal 0 to MCU
        elif received_data == '5':
            is_blink = False
            ser.write(b'0')  # # After closing the door, return signal 0 to MCU
        time.sleep(0.01)




