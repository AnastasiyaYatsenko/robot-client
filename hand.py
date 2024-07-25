# import RPi.GPIO as GPIO
import RPi.GPIO as GPIO
import os
import serial
from time import sleep          # this lets us have a time delay
from typing import NamedTuple
from struct import *
import sys
import logging


class Params(NamedTuple):
    lin_step: float
    ang_step: float
    PoT_lin: int
    PoT_ang: int
    lin_mm: float
    ang_deg: float
    hold: int


#BOOT0 = 19  # gpio19
#RESET = 26  # gpio26

# TODO check pin numbers
def setupGPIO():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(27, GPIO.OUT) #BOOT0 for stm n.1
    GPIO.setup(22, GPIO.OUT) #RESET for stm n.1

    GPIO.output(27, 0) # Normal flash boot
    GPIO.output(22, 0) # Normal running state
    
    GPIO.setup(23, GPIO.OUT) #BOOT0 for stm n.2
    GPIO.setup(19, GPIO.OUT) #RESET for stm n.2

    GPIO.output(23, 0) # Normal flash boot
    GPIO.output(19, 0) # Normal running state
    
    GPIO.setup(20, GPIO.OUT) #BOOT0 for stm n.3
    GPIO.setup(16, GPIO.OUT) #RESET for stm n.3

    GPIO.output(20, 0) # Normal flash boot
    GPIO.output(16, 0) # Normal running state


class Hand:

    def __init__(self, tty, br, num, boot_pin, reset_pin):
        self.tty = tty
        self.br = br
        self.ser = serial.Serial(tty, br)
        self.ser.timeout = 0.1
        self.params = Params(0.0, 0.0, 0, 0, 0.0, 0.0, 0)
        self.num = num
        self.BOOT0 = boot_pin
        self.RESET = reset_pin

    def read_serial(self, bytes):
        try:
            tdata = self.ser.read(bytes)
            # logging.error(f"DATA FROM STM {self.num}: {binascii.hexlify(tdata, ':', 2)}")
            # print("in read")
        except serial.SerialException as e:
            # There is no new data from serial port
            logging.error(f"{self.num}: SerialException")
            return None
        except TypeError as e:
            # Disconnect of USB->UART occured
            logging.error("TypeError")
            self.ser.close()
            return None
        else:
            # Some data was received
            # print("data received")
            return tdata

    def stop(self, com_flag, results, i):
        print("stop")
        wait_count = 0
        while not com_flag and wait_count < 10:
            wait_count += 1
            sleep(0.1)
        if wait_count >= 10:
            logging.error(f'Port on hand {self.num} is occupied for too long')
            results[i] = -1
            return -1
        com_flag = False
        logging.error("Stop")
        p = pack('@ffiiffi', 0.0, 0.0, 0, 0, 0.0, 0.0, 25)
        print(p)
        # logging.error("send: " + str(p) + "\n")
        self.ser.write(p)

        # tdata = self.ser.read(12)
        tdata = self.read_serial(28)
        if (tdata):
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            if LS2[6] != 10:
                logging.error("Some error occured while stopping the hand " + str(self.num))
                com_flag = True
                results[i] = -1
                return -1
            com_flag = True
            results[i] = 1
            return 1
        else:
            logging.error("No response from the hand " + str(self.num))
            com_flag = True
            results[i] = -1
            return -1

    def start(self, com_flag, results, i):
        wait_count = 0
        while not com_flag and wait_count < 10:
            wait_count += 1
            sleep(0.1)
        if wait_count >= 10:
            logging.error(f'Port on hand {self.num} is occupied for too long')
            results[i] = -1
            return -1
        com_flag = False
        print("start")
        l = self.params.lin_mm
        a = self.params.ang_deg
        h = self.params.hold
        print(l)
        # https://docs.python.org/3/library/struct.html#examples
        # Format Characters
        # @  === native
        # f  === float
        # i  === int
        p = pack('@ffiiffi', 0.0, 0.0, 0, 0, l, a, h)
        # logging.error("send: " + str(p) + "\n")
        print(p)

        self.ser.write(p)

        # return 1
        '''self.ser.timeout = 30
        #tdata = self.ser.read(12)
        tdata = self.read_serial(28)
        #self.ser.timeout = 0.5
        if tdata:
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            if not(LS2[6] == 10 or LS2[6] == 11):
                logging.error("Some error occured while moving the hand "+str(self.num))
                com_flag = True
                results[i] = [-1]
                return -1
            com_flag = True
            #print('okie!')
            #logging.error(f'Hand {self.num} | lin: {LS2[4]:.3f}; ang: {LS2[5]:.3f}; hold: {LS2[6]}\n')
            results[i] = LS2
            return 1
        else:
            logging.error("No response from the hand "+str(self.num))
            com_flag = True
            results[i] = [-1]
            return -1'''

        self.ser.timeout = 30
        # tdata = self.ser.read(12)
        tdata = self.read_serial(28)
        self.ser.timeout = 0.5
        if tdata:
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            # logging.error(f"{LS2[6]}")
            if not (LS2[6] == 10 or LS2[6] == 11):
                logging.error("Some error occured while moving the hand " + str(self.num))
                com_flag = True
                results[i] = [-1]
                return -1
            # logging.error(f'Hand {self.num} | lin: {LS2[4]:.3f}; ang: {LS2[5]:.3f}; hold: {LS2[6]}\n')
            com_flag = True
            # print('okie!')
            results[i] = LS2
            return 1
        else:
            logging.error("No response from the hand " + str(self.num))
            com_flag = True
            results[i] = [-1]
            return -1

    def setSteps(self, com_flag, results, i):
        wait_count = 0
        while not com_flag and wait_count < 10:
            wait_count += 1
            sleep(0.1)
        if wait_count >= 10:
            logging.error(f'Port on hand {self.num} is occupied for too long')
            results[i] = -1
            return -1
        com_flag = False
        print("start")
        l = self.params.lin_step
        a = self.params.ang_step
        pot_l = self.params.PoT_lin
        pot_a = self.params.PoT_ang
        l_mm = self.params.lin_mm
        a_deg = self.params.ang_deg
        h = 30 + self.params.hold
        # print(l)
        # https://docs.python.org/3/library/struct.html#examples
        # Format Characters
        # @  === native
        # f  === float
        # i  === int
        p = pack('@ffiiffi', l, a, pot_l, pot_a, l_mm, a_deg, h)
        # logging.error(f"Hand {self.num} send {l} {a} {pot_l} {pot_a} {h}\n")
        # logging.error(F"Hand {self.num} send {p}\n")
        print(p)

        self.ser.write(p)

        # return 1
        self.ser.timeout = 5
        # tdata = self.ser.read(12)
        tdata = self.read_serial(28)
        self.ser.timeout = 0.5
        if tdata:
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            if LS2[6] != 10:
                logging.error("Some error occured while moving the hand " + str(self.num))
                com_flag = True
                results[i] = -1
                return -1
            com_flag = True
            # print('okie!')
            results[i] = 1
            return 1
        else:
            logging.error("No response from the hand " + str(self.num))
            com_flag = True
            results[i] = -1
            return -1

    def startSteps(self, com_flag, results, i):
        wait_count = 0
        while not com_flag and wait_count < 10:
            wait_count += 1
            sleep(0.1)
        if wait_count >= 10:
            logging.error(f'Port on hand {self.num} is occupied for too long')
            results[i] = -1
            return -1
        com_flag = False
        print("start")
        h = 40
        # print(l)
        # https://docs.python.org/3/library/struct.html#examples
        # Format Characters
        # @  === native
        # f  === float
        # i  === int
        p = pack('@ffiiffi', 0.0, 0.0, 0, 0, 0.0, 0.0, h)
        # logging.error(f"send {l} {a} {pot_l} {pot_a} {h}\n")
        # logging.error(f"START {self.num} send: {p}\n")
        print(p)

        self.ser.write(p)

        # return 1
        '''self.ser.timeout = 30
        #tdata = self.ser.read(12)
        tdata = self.read_serial(28)
        #self.ser.timeout = 0.5
        if tdata:
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            #logging.error(f"{LS2[6]}")
            if not(LS2[6] == 10 or LS2[6] == 11):
                logging.error("Some error occured while moving the hand "+str(self.num))
                com_flag = True
                results1[i] = [-1]
                return -1
            #logging.error(f'Hand {self.num} | lin: {LS2[4]:.3f}; ang: {LS2[5]:.3f}; hold: {LS2[6]}\n')
            com_flag = True
            #print('okie!')
            results1[i] = LS2
            return 1
        else:
            logging.error("No response from the hand "+str(self.num))
            com_flag = True
            results[i] = [-1]
            return -1'''

        self.ser.timeout = 30
        # tdata = self.ser.read(12)
        tdata = self.read_serial(28)
        self.ser.timeout = 0.5
        if tdata:
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            #            logging.error(f"{self.num} {LS2}")
            if not (LS2[6] == 10 or LS2[6] == 11):
                logging.error("Some error occured while moving the hand " + str(self.num))
                com_flag = True
                results[i] = [-1]
                return -1
            # logging.error(f'Hand {self.num} | lin: {LS2[4]:.3f}; ang: {LS2[5]:.3f}; hold: {LS2[6]}\n')
            com_flag = True
            # print('okie!')
            results[i] = LS2
            return 1
        else:
            logging.error("No response from the hand " + str(self.num))
            com_flag = True
            results[i] = [-1]
            return -1

    def get(self, com_flag, results, i, v=True):
        # flag = False
        print("in get")
        wait_count = 0
        while not com_flag and wait_count < 10:
            wait_count += 1
            sleep(0.1)
        if wait_count >= 10:
            logging.error(f'Port on hand {self.num} is occupied for too long')
            results[i] = [-1]
            return -1
        com_flag = False
        self.clear_inWaiting()
        p = pack('@ffiiffi', 1.0, 1.0, 1, 1, 1.0, 1.0, 50)
        # logging.error("send: " + str(p))
        print(p)

        self.ser.write(p)
        # tdata = self.ser.read(12)  # Wait forever for anything
        tdata = self.read_serial(28)
        print("------")
        # strdata = ":".join("{:02x}".format(ord(c)) for c in tdata)
        #        logging.error(f"DATA FROM STM {self.num}: {binascii.hexlify(tdata, ':', 2)}")
        print("---")
        if tdata:
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            if v:
                logging.error(f'Hand {self.num} | lin: {LS2[4]:.3f}; ang: {LS2[5]:.3f}; hold: {LS2[6]}\n')
            # logging.error("Hand "+str(self.num)+" | lin: "+str(LS2[0])+"; ang: "+str(LS2[1])+"; hold: "+str(LS2[2])+ "\n")
            # print(f'Hand {self.num} | lin: {LS2[4]:.3f}; ang: {LS2[5]:.3f}; hold: {LS2[6]}')
            # print("Hand "+str(self.num)+" | lin: "+str(LS2[0])+"; ang: "+str(LS2[1])+"; hold: "+str(LS2[2]))
            com_flag = True
            results[i] = LS2
            return LS2
        else:
            logging.error("Timeout reading Serial (Hand " + str(self.num) + ")")
            print("Timeout reading Serial (Hand " + str(self.num) + ")")
            com_flag = True
            results[i] = [-1]
            return [-1]

    def getVersion(self, com_flag, results, i):
        # flag = False
        print("in get Version")
        wait_count = 0
        while not com_flag and wait_count < 10:
            wait_count += 1
            sleep(0.1)
        if wait_count >= 10:
            logging.error(f'Port on hand {self.num} is occupied for too long')
            results[i] = [-1]
            return -1
        com_flag = False
        self.clear_inWaiting()
        p = pack('@ffiiffi', 0.0, 0.0, 0, 0, 0.0, 0.0, 80)
        # logging.error("send: " + str(p))
        print(p)

        self.ser.write(p)
        # tdata = self.ser.read(12)  # Wait forever for anything
        tdata = self.read_serial(28)
        print("------")
        print(tdata)
        print("---")
        if tdata:
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            logging.error(f'Hand {self.num} | version: {LS2[6]}\n')
            # logging.error("Hand "+str(self.num)+" | lin: "+str(LS2[0])+"; ang: "+str(LS2[1])+"; hold: "+str(LS2[2])+ "\n")
            print(f'Hand {self.num} | version: {LS2[6]}\n')
            # print("Hand "+str(self.num)+" | lin: "+str(LS2[0])+"; ang: "+str(LS2[1])+"; hold: "+str(LS2[2]))
            com_flag = True
            results[i] = LS2[6]
            return LS2
        else:
            logging.error("Timeout reading Serial (Hand " + str(self.num) + ")")
            print("Timeout reading Serial (Hand " + str(self.num) + ")")
            com_flag = True
            results[i] = [-1]
            return [-1]

    # def setPos(self, l, a, h):
    # not used
    # print("set zero position")
    # send cmd to stm to set proper zero
    # self.params = Params(l, a, h)

    def setZeroPos(self, com_flag, results, i):
        wait_count = 0
        while not com_flag and wait_count < 10:
            wait_count += 1
            sleep(0.1)
        if wait_count >= 10:
            logging.error(f'Port on hand {self.num} is occupied for too long')
            results[i] = -1
            return -1
        com_flag = False
        logging.error("Set zero position")
        # print("! set zero position")
        self.clear_inWaiting()
        p = pack('@ffiiffi', 0.0, 0.0, 0, 0, 0.0, 0.0, 75)
        # logging.error("send: " + str(p) + "\n")
        print(p)
        self.ser.write(p)

        # tdata = self.ser.read(12)
        tdata = self.read_serial(28)
        if tdata:
            print(tdata)
            unpacked_struct = unpack('@ffiiffi', tdata)
            LS2 = list(unpacked_struct)
            if LS2[6] != 10:
                logging.error("an error occurred when setting zero position to the hand " + str(self.num))
                print("an error occurred when setting zero position to the hand " + str(self.num))
                com_flag = True
                results[i] = -1
                return -1
        else:
            print("No answer recieved from hand " + str(self.num))
            com_flag = True
            results[i] = -1
            return -1

    def flash(self, url, com_flag, results, i):
        wait_count = 0
        while not com_flag and wait_count < 10:
            wait_count += 1
            sleep(0.1)
        if wait_count >= 10:
            logging.error(f'Port on hand {self.num} is occupied for too long')
            results[i] = -1
            return -1
        com_flag = False
        print("flash")
        logging.error("Flash")
        self.clear_inWaiting()
        self.ser.close()
        '''p = pack('@ffi', 0.0, 0.0, 25)
        logging.error("send: " + str(p))
        logging.error("")
        print(p)
        self.ser.write(p)

        tdata = self.ser.read(12)
        unpacked_struct = unpack('@ffi', tdata)
        LS2 = list(unpacked_struct)
        if LS2[2]!=10:
            logging.error("an error occurred when stopping the hand")
            print("an error occurred when stopping the hand")
            return -1'''
        # url = "https://github.com/AnastasiyaYatsenko/robot_bin/blob/main/led_toggle.bin?raw=true" #TEST .BIN
        os.system("wget -q -O stmfirmware" + str(self.num) + ".bin " + url)

        bin_file = "stmfirmware" + str(self.num) + ".bin"
        print(bin_file)
        GPIO.output(self.BOOT0, 1)  # Boot from serial
        GPIO.output(self.RESET, 1)  # reset
        sleep(0.1)
        GPIO.output(self.RESET, 0)
        sleep(0.5)

        os.system(str("stm32flash " + self.tty))
        sleep(1)
        # cmd = str("stm32flash -b 115200 -w stmfirmware"+str(self.num)+".bin "+self.tty)
        cmd = str("stm32flash -w stmfirmware" + str(self.num) + ".bin " + self.tty)
        logging.error(cmd)
        os.system(cmd)
        #        os.system(str("stm32flash -b 115200 -w stmfirmware"+str(self.num)+".bin "+self.tty))
        # stm32flash -w stmfirmware15.bin /dev/ttyAMA5

        sleep(0.1)
        GPIO.output(self.BOOT0, 0)  # boot from flash
        sleep(0.1)
        GPIO.output(self.RESET, 1)  # reset
        sleep(0.1)
        GPIO.output(self.RESET, 0)
        self.ser.open()
        com_flag = True
        results[i] = 1

    def reset(self, results, i):
        print("reset")
        logging.error("Reset")

        GPIO.output(self.RESET, 1)  # reset
        sleep(0.1)
        GPIO.output(self.RESET, 0)
        sleep(0.5)

        results[i] = 1

    # def setAngDir(self, dir):
    #     if dir == 0:
    #         # disable inverse
    #         p = pack('@ffiii', 0.0, 0.0, 0, 0, 30)
    #         self.ser.write(p)
    #     elif dir == 1:
    #         # enable inverse
    #         p = pack('@ffiii', 0.0, 0.0, 0, 0, 31)
    #         self.ser.write(p)
    #
    # def setLinDir(self, dir):
    #     if dir == 0:
    #         # disable inverse
    #         p = pack('@ffiii', 0.0, 0.0, 0, 0, 40)
    #         self.ser.write(p)
    #     elif dir == 1:
    #         # enable inverse
    #         p = pack('@ffiii', 0.0, 0.0, 0, 0, 41)
    #         self.ser.write(p)

    def clear_inWaiting(self):
        if self.ser.inWaiting():
            self.read_serial(self.ser.inWaiting())

