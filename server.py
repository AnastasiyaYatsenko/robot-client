#!/usr/bin/env python3
from math import ceil

# import RPi.GPIO as GPIO
import RPi.GPIO as GPIO
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
import socketserver
from urllib.parse import urlparse
from urllib.parse import parse_qs
import re
from urllib.parse import urlparse, parse_qs
import logging
import threading
from hand import *

loglevel = logging.DEBUG
logfile  = 'log_python.txt'


class Robot:
    robot = [Hand('/dev/ttyAMA5', 115200, 15, 27, 22),
             Hand('/dev/ttyAMA3', 115200, 17, 23, 19),
             Hand('/dev/ttyAMA4', 115200, 19, 20, 16)]
             #Hand('/dev/ttyAMA4', 115200, 19, 20, 16)]
    
    # COM occupied flags
    hand_com = [True, True, True]
    allow_unhold = [0]
    holders = [0, 0, 0]
    on_start_params = [0]
    
    # result variables
    start_results = [[], [], []]
    set_results = [0, 0, 0]
    stop_results = [0, 0, 0]
    flash_results = [0, 0, 0]
    reset_results = [0, 0, 0]
    setZero_results = [0, 0, 0]
    get_results = [[], [], []]
    getVersion_results = [0, 0, 0]
    
    def set(self, i, lin, ang, hold):
        self.robot[i].params = Params(0.0, 0.0, 0, 0, lin, ang, hold)
        print(str(self.robot[i].params.lin_mm) + " " + str(self.robot[i].params.ang_deg) + " " + str(self.robot[i].params.hold))

    def get(self):
        t_get1 = threading.Thread(target=self.robot[0].get, args=(self.hand_com[0], self.get_results, 0))
        t_get2 = threading.Thread(target=self.robot[1].get, args=(self.hand_com[1], self.get_results, 1))
        t_get3 = threading.Thread(target=self.robot[2].get, args=(self.hand_com[2], self.get_results, 2))
        t_get1.start()
        t_get2.start()
        t_get3.start()
        t_get1.join()
        t_get2.join()
        t_get3.join()

    def get_hand(self, i):
        t_get1 = threading.Thread(target=self.robot[i].get, args=(self.hand_com[i], self.get_results, i))
        t_get1.start()

    def send(self, i):
        self.get_params_holders()
        non_i = [0, 1, 2]
        non_i.remove(i)
        if (self.holders[non_i[0]] != 1 or self.holders[non_i[1]] != 1) and not self.allow_unhold[0] and \
                self.robot[i].params[6] == 0:
            logging.error(f'Another hand is unholded already!')
        else:
            t_start1 = threading.Thread(target=self.robot[i].start, args=(self.hand_com[i], self.start_results, i))
            t_start1.start()

    def sendSteps(self):#, lin1, ang1, hold1, lin2, ang2, hold2, lin3, ang3, hold3):
        steps_periods = self.calc_steps_and_ARR(self.robot[0].params.ang_deg, self.robot[0].params.lin_mm,
                                                self.robot[1].params.ang_deg, self.robot[1].params.lin_mm,
                                                self.robot[2].params.ang_deg, self.robot[2].params.lin_mm)

        self.robot[0].params = Params(steps_periods[0][0], steps_periods[0][1], steps_periods[1][0],
                                      steps_periods[1][1], self.robot[0].params.lin_mm, self.robot[0].params.ang_deg,
                                      self.robot[0].params.hold)
        self.robot[1].params = Params(steps_periods[0][2], steps_periods[0][3], steps_periods[1][2],
                                      steps_periods[1][3], self.robot[1].params.lin_mm, self.robot[1].params.ang_deg,
                                      self.robot[1].params.hold)
        self.robot[2].params = Params(steps_periods[0][4], steps_periods[0][5], steps_periods[1][4],
                                      steps_periods[1][5], self.robot[2].params.lin_mm, self.robot[2].params.ang_deg,
                                      self.robot[2].params.hold)

        t_set1 = threading.Thread(target=self.robot[0].setSteps,
                                  args=(self.hand_com[0], self.set_results, 0))
        t_set2 = threading.Thread(target=self.robot[1].setSteps,
                                  args=(self.hand_com[1], self.set_results, 1))
        t_set3 = threading.Thread(target=self.robot[2].setSteps,
                                  args=(self.hand_com[2], self.set_results, 2))
        t_set1.start()
        t_set2.start()
        t_set3.start()

        logging.error(
            f'SEND  : ({self.robot[0].params.lin_mm:.3f}, {self.robot[0].params.ang_deg:.3f}), '
            f'({self.robot[1].params.lin_mm:.3f}, {self.robot[1].params.ang_deg:.3f}), '
            f'({self.robot[2].params.lin_mm:.3f}, {self.robot[2].params.ang_deg:.3f})')

        if (self.set_results[0] < 0) or (self.set_results[1] < 0) or (self.set_results[2] < 0):
            logging.error(
                f'Error occurred on setting coordinates: ({self.robot[0].params.lin_mm}, {self.robot[0].params.ang_deg}),'
                f' ({self.robot[1].params.lin_mm}, {self.robot[1].params.ang_deg}),'
                f' ({self.robot[2].params.lin_mm}, {self.robot[2].params.ang_deg})\nAbort the move')
            return

        # wait for threads to end AND for robot to finish move

        t_set1.join()
        t_set2.join()
        t_set3.join()

        t_start1 = threading.Thread(target=self.robot[0].startSteps,
                                    args=(self.hand_com[0], self.start_results, 0))
        t_start2 = threading.Thread(target=self.robot[1].startSteps,
                                    args=(self.hand_com[1], self.start_results, 1))
        t_start3 = threading.Thread(target=self.robot[2].startSteps,
                                    args=(self.hand_com[2], self.start_results, 2))
        # logging.error("BEFORE start")
        t_start1.start()
        t_start2.start()
        t_start3.start()
        # logging.error("AFTER start")

        # sleep(1)

        # wait for threads to end AND for robot to finish move

        t_start1.join()
        t_start2.join()
        t_start3.join()

        if (self.start_results[0][0] < 0) or (self.start_results[1][0] < 0) or (self.start_results[2][0] < 0):
            logging.error(
                f'Error occurred on setting coordinates: ({self.robot[0].params.lin_mm}, {self.robot[0].params.ang_deg}),'
                f' ({self.robot[1].params.lin_mm}, {self.robot[1].params.ang_deg}),'
                f' ({self.robot[2].params.lin_mm}, {self.robot[2].params.ang_deg})\nAbort the move')
            return

        logging.error(
            f"FIRST : ({self.start_results[0][0]:.3f}, {self.start_results[0][1]:.3f}), ({self.start_results[1][0]:.3f},"
            f" {self.start_results[1][1]:.3f}), ({self.start_results[2][0]:.3f}, {self.start_results[2][1]:.3f})")
        logging.error(
            f"AFTER : ({self.start_results[0][4]:.3f}, {self.start_results[0][5]:.3f}), ({self.start_results[1][4]:.3f},"
            f" {self.start_results[1][5]:.3f}), ({self.start_results[2][4]:.3f}, {self.start_results[2][5]:.3f})")
        logging.error("---")
        # logging.error(f"DELTA : ({abs(self.get_results[0][4]-shift0[i]):.3f}, {abs(self.get_results[0][5]-angle0[i]):.3f}), ({abs(self.get_results[1][4]-shift1[i]):.3f}, {abs(self.get_results[1][5]-angle1[i]):.3f}), ({abs(self.get_results[2][4]-shift2[i]):.3f}, {abs(self.get_results[2][5]-angle2[i]):.3f})")
        delta_l1 = self.start_results[0][4] - self.robot[0].params.lin_mm
        delta_a1 = self.start_results[0][5] - self.robot[0].params.ang_deg
        delta_l2 = self.start_results[1][4] - self.robot[1].params.lin_mm
        delta_a2 = self.start_results[1][5] - self.robot[1].params.ang_deg
        delta_l3 = self.start_results[2][4] - self.robot[2].params.lin_mm
        delta_a3 = self.start_results[2][5] - self.robot[2].params.ang_deg
        if (360.0 - delta_a1 < delta_a1):
            delta_a1 = 360.0 - delta_a1
        if (360.0 - delta_a2 < delta_a2):
            delta_a2 = 360.0 - delta_a2
        if (360.0 - delta_a3 < delta_a3):
            delta_a3 = 360.0 - delta_a3
        logging.error(
            f"DELTA : ({delta_l1:.3f}, {delta_a1:.3f}), ({delta_l2:.3f}, {delta_a2:.3f}), ({delta_l3:.3f}, {delta_a3:.3f})")
        logging.error("---------------------")

        return 1

    def hold(self):
        self.get_params_holders()
        t_get1 = threading.Thread(target=self.robot[0].get,
                                  args=(self.hand_com[0], self.get_results, 0))
        t_get2 = threading.Thread(target=self.robot[1].get,
                                  args=(self.hand_com[1], self.get_results, 1))
        t_get3 = threading.Thread(target=self.robot[2].get,
                                  args=(self.hand_com[2], self.get_results, 2))
        t_get1.start()
        t_get2.start()
        t_get3.start()

        t_get1.join()
        t_get2.join()
        t_get3.join()
        # logging.error(f"Get results: ({self.get_results[0][4]:.3f}, {self.get_results[0][5]:.3f}), ({self.get_results[1][4]:.3f}, {self.get_results[1][5]:.3f}), ({self.get_results[2][4]:.3f}, {self.get_results[2][5]:.3f})")
        error_flag = False

        if (self.get_results[0][0] < 0) or (self.get_results[1][0] < 0) or (self.get_results[2][0] < 0):
            logging.error(f'Error occurred while getting the coordinates\nAbort the move')
            error_flag = True

        if not error_flag:
            lin_1 = self.get_results[0][4]
            ang_1 = self.get_results[0][5]

            lin_2 = self.get_results[1][4]
            ang_2 = self.get_results[1][5]

            lin_3 = self.get_results[2][4]
            ang_3 = self.get_results[2][5]

            self.robot[0].params = Params(0, 0, 0, 0, lin_1, ang_1, 1)
            self.robot[1].params = Params(0, 0, 0, 0, lin_2, ang_2, 1)
            self.robot[2].params = Params(0, 0, 0, 0, lin_3, ang_3, 1)

            t_start1 = threading.Thread(target=self.robot[0].start,
                                        args=(self.hand_com[0], self.start_results, 0))
            t_start2 = threading.Thread(target=self.robot[1].start,
                                        args=(self.hand_com[1], self.start_results, 1))
            t_start3 = threading.Thread(target=self.robot[2].start,
                                        args=(self.hand_com[2], self.start_results, 2))

            t_start1.start()
            t_start2.start()
            t_start3.start()

            t_start1.join()
            t_start2.join()
            t_start3.join()

    def unhold(self):
        self.get_params_holders()
        if not self.allow_unhold[0]:
            logging.error(f'Unhold is not allowed!')
        else:
            t_get1 = threading.Thread(target=self.robot[0].get,
                                      args=(self.hand_com[0], self.get_results, 0))
            t_get2 = threading.Thread(target=self.robot[1].get,
                                      args=(self.hand_com[1], self.get_results, 1))
            t_get3 = threading.Thread(target=self.robot[2].get,
                                      args=(self.hand_com[2], self.get_results, 2))
            t_get1.start()
            t_get2.start()
            t_get3.start()

            t_get1.join()
            t_get2.join()
            t_get3.join()
            # logging.error(f"Get results: ({self.get_results[0][4]:.3f}, {self.get_results[0][5]:.3f}), ({self.get_results[1][4]:.3f}, {self.get_results[1][5]:.3f}), ({self.get_results[2][4]:.3f}, {self.get_results[2][5]:.3f})")
            error_flag = False

            if (self.get_results[0][0] < 0) or (self.get_results[1][0] < 0) or (self.get_results[2][0] < 0):
                logging.error(f'Error occurred while getting the coordinates\nAbort the move')
                error_flag = True

            if not error_flag:
                lin_1 = self.get_results[0][4]
                ang_1 = self.get_results[0][5]

                lin_2 = self.get_results[1][4]
                ang_2 = self.get_results[1][5]

                lin_3 = self.get_results[2][4]
                ang_3 = self.get_results[2][5]

                self.robot[0].params = Params(0, 0, 0, 0, lin_1, ang_1, 0)
                self.robot[1].params = Params(0, 0, 0, 0, lin_2, ang_2, 0)
                self.robot[2].params = Params(0, 0, 0, 0, lin_3, ang_3, 0)

                t_start1 = threading.Thread(target=self.robot[0].start,
                                            args=(self.hand_com[0], self.start_results, 0))
                t_start2 = threading.Thread(target=self.robot[1].start,
                                            args=(self.hand_com[1], self.start_results, 1))
                t_start3 = threading.Thread(target=self.robot[2].start,
                                            args=(self.hand_com[2], self.start_results, 2))

                t_start1.start()
                t_start2.start()
                t_start3.start()

                t_start1.join()
                t_start2.join()
                t_start3.join()

    def getVersion(self, i):
        t_get1 = threading.Thread(target=self.robot[i].getVersion, args=(self.hand_com[i], self.getVersion_results, i))
        t_get1.start()

    def stop(self):
        t_stop1 = threading.Thread(target=self.robot[0].stop, args=(self.hand_com[0], self.stop_results, 0))
        t_stop2 = threading.Thread(target=self.robot[1].stop, args=(self.hand_com[1], self.stop_results, 1))
        t_stop3 = threading.Thread(target=self.robot[2].stop, args=(self.hand_com[2], self.stop_results, 2))

        t_stop1.start()
        t_stop2.start()
        t_stop3.start()

    def zeroPos(self):
        t_zero1 = threading.Thread(target=self.robot[0].setZeroPos, args=(self.hand_com[0], self.setZero_results, 0))
        t_zero2 = threading.Thread(target=self.robot[1].setZeroPos, args=(self.hand_com[1], self.setZero_results, 1))
        t_zero3 = threading.Thread(target=self.robot[2].setZeroPos, args=(self.hand_com[2], self.setZero_results, 2))

        t_zero1.start()
        t_zero2.start()
        t_zero3.start()

    def flash(self):
        t_flash1 = threading.Thread(target=self.robot[0].flash, args=(
        "https://github.com/AnastasiyaYatsenko/robot_bin/blob/main/hand_0.bin?raw=true", self.hand_com[0],
        self.flash_results, 0))
        t_flash2 = threading.Thread(target=self.robot[1].flash, args=(
        "https://github.com/AnastasiyaYatsenko/robot_bin/blob/main/hand_1.bin?raw=true", self.hand_com[1],
        self.flash_results, 1))
        t_flash3 = threading.Thread(target=self.robot[2].flash, args=(
        "https://github.com/AnastasiyaYatsenko/robot_bin/blob/main/hand_2.bin?raw=true", self.hand_com[2],
        self.flash_results, 2))

        t_flash1.start()
        t_flash2.start()
        t_flash3.start()

    def reset(self):
        t_reset1 = threading.Thread(target=self.robot[0].reset, args=(self.reset_results, 0))
        t_reset2 = threading.Thread(target=self.robot[1].reset, args=(self.reset_results, 1))
        t_reset3 = threading.Thread(target=self.robot[2].reset, args=(self.reset_results, 2))

        t_reset1.start()
        t_reset2.start()
        t_reset3.start()

    def get_params_holders(self):
        # logging.error(f'Get params on start (inside): {self.on_start_params[0]}')
        # if self.on_start_params:
        #     return
        self.on_start_params[0] = 1
        # logging.error(f'params on start: {self.on_start_params}')
        t_get1 = threading.Thread(target=self.robot[0].get, args=(self.hand1_com[0], self.get_results, 0, False))
        t_get2 = threading.Thread(target=self.robot[1].get, args=(self.hand2_com[0], self.get_results, 1, False))
        t_get3 = threading.Thread(target=self.robot[2].get, args=(self.hand3_com[0], self.get_results, 2, False))

        t_get1.start()
        t_get2.start()
        t_get3.start()

        t_get1.join()
        t_get2.join()
        t_get3.join()

        self.holders[0] = self.get_results[0][6]
        self.holders[1] = self.get_results[1][6]
        self.holders[2] = self.get_results[2][6]

    def allow(self):
        if not self.allow_unhold[0]:
            logging.error(f'Unhold is allowed')
            self.allow_unhold[0] = 1
        else:
            logging.error(f'Unhold is forbidden')
            self.allow_unhold[0] = 0

    # TODO
    def oneR(self):
        print("1R")
        f = open("arr.txt", "r")
        file_arr = f.read()

        a0_start = file_arr.find("angle0")+10
        a0_end = file_arr.find("}", a0_start)
        angle0_str = file_arr[a0_start:a0_end].split(",")

        s0_start = file_arr.find("shift0") + 10
        s0_end = file_arr.find("}", s0_start)
        shift0_str = file_arr[s0_start:s0_end].split(",")

        a1_start = file_arr.find("angle1") + 10
        a1_end = file_arr.find("}", a1_start)
        angle1_str = file_arr[a1_start:a1_end].split(",")

        s1_start = file_arr.find("shift1") + 10
        s1_end = file_arr.find("}", s1_start)
        shift1_str = file_arr[s1_start:s1_end].split(",")

        a2_start = file_arr.find("angle2") + 10
        a2_end = file_arr.find("}", a2_start)
        angle2_str = file_arr[a2_start:a2_end].split(",")

        s2_start = file_arr.find("shift2") + 10
        s2_end = file_arr.find("}", s2_start)
        shift2_str = file_arr[s2_start:s2_end].split(",")

        h0_start = file_arr.find("hoock0") + 10
        h0_end = file_arr.find("}", h0_start)
        hoock0_str = file_arr[h0_start:h0_end].split(",")

        h1_start = file_arr.find("hoock1") + 10
        h1_end = file_arr.find("}", h1_start)
        hoock1_str = file_arr[h1_start:h1_end].split(",")

        h2_start = file_arr.find("hoock2") + 10
        h2_end = file_arr.find("}", h2_start)
        hoock2_str = file_arr[h2_start:h2_end].split(",")

        arr_len = len(angle0_str)

        angle0 = []
        shift0 = []
        angle1 = []
        shift1 = []
        angle2 = []
        shift2 = []
        hoock0 = []
        hoock1 = []
        hoock2 = []

        counter = -1
        last_hoock0 = int(hoock0_str[0])
        last_hoock1 = int(hoock1_str[0])
        last_hoock2 = int(hoock2_str[0])

        for i in range(arr_len):
            counter += 1
            h0 = int(hoock0_str[i])
            h1 = int(hoock1_str[i])
            h2 = int(hoock2_str[i])
            if (counter == 99) or (h0 != last_hoock0) or (h1 != last_hoock1) or (h2 != last_hoock2):
                angle0.append(float(angle0_str[i]))
                shift0.append(float(shift0_str[i]))
                angle1.append(float(angle1_str[i]))
                shift1.append(float(shift1_str[i]))
                angle2.append(float(angle2_str[i]))
                shift2.append(float(shift2_str[i]))
                hoock0.append(int(hoock0_str[i]))
                hoock1.append(int(hoock1_str[i]))
                hoock2.append(int(hoock2_str[i]))
                counter = -1
            last_hoock0 = h0
            last_hoock1 = h1
            last_hoock2 = h2
        f.close()

        for i in range(0, arr_len):
            steps_periods = self.calc_steps_and_ARR(angle0[i], shift0[i], angle1[i], shift1[i], angle2[i], shift2[i])

            # logging.error(f"Send params: ({shift0[i]}, {angle0[i]}), ({shift1[i]}, {angle1[i]}), ({shift2[i]}, {angle2[i]})")
            self.robot[0].params = Params(steps_periods[0][0], steps_periods[0][1], steps_periods[1][0],
                                          steps_periods[1][1], hoock0[i])
            self.robot[1].params = Params(steps_periods[0][2], steps_periods[0][3], steps_periods[1][2],
                                          steps_periods[1][3], hoock1[i])
            self.robot[2].params = Params(steps_periods[0][4], steps_periods[0][5], steps_periods[1][4],
                                          steps_periods[1][5], hoock2[i])

            '''self.robot[0].params = Params(shift0[i], angle0[i], 0, 0, hoock0[i])
            self.robot[1].params = Params(shift1[i], angle1[i], 0, 0, hoock1[i])
            self.robot[2].params = Params(shift2[i], angle2[i], 0, 0, hoock2[i])'''

            '''self.robot[0].setSteps(self.hand1_com, self.set_results, 0)
            self.robot[1].setSteps(self.hand2_com, self.set_results, 1)
            self.robot[2].setSteps(self.hand3_com, self.set_results, 2)'''
            t_set1 = threading.Thread(target=self.robot[0].setSteps,
                                      args=(self.hand_com[0], self.set_results, 0))
            t_set2 = threading.Thread(target=self.robot[1].setSteps,
                                      args=(self.hand_com[1], self.set_results, 1))
            t_set3 = threading.Thread(target=self.robot[2].setSteps,
                                      args=(self.hand_com[2], self.set_results, 2))
            t_set1.start()
            t_set2.start()
            t_set3.start()

            if (self.set_results[0] < 0) or (self.set_results[1] < 0) or (self.set_results[2] < 0):
                logging.error(
                    f'Error occurred on setting coordinates: ({shift0[i]}, {angle0[i]}), ({shift1[i]}, {angle1[i]}), ({shift2[i]}, {angle2[i]})\nAbort the move')
                break

            '''status0 = self.robot[0].start(self.hand1_com, self.start_results, 0)
            status1 = self.robot[1].start(self.hand2_com, self.start_results, 1)
            status2 = self.robot[2].start(self.hand3_com, self.start_results, 2)'''
            # sleep(1)

            # wait for threads to end AND for robot to finish move

            t_set1.join()
            t_set2.join()
            t_set3.join()
            # logging.error("GET obtained")

            t_start1 = threading.Thread(target=self.robot[0].startSteps,
                                        args=(self.hand_com[0], self.start_results, 0))
            t_start2 = threading.Thread(target=self.robot[1].startSteps,
                                        args=(self.hand_com[1], self.start_results, 1))
            t_start3 = threading.Thread(target=self.robot[2].startSteps,
                                        args=(self.hand_com[2], self.start_results, 2))
            # logging.error("BEFORE start")
            t_start1.start()
            t_start2.start()
            t_start3.start()
            # logging.error("AFTER start")

            # sleep(1)

            # wait for threads to end AND for robot to finish move

            t_start1.join()
            t_start2.join()
            t_start3.join()

            '''self.robot[0].startSteps(self.hand1_com, self.start_results, 0)
            self.robot[1].startSteps(self.hand2_com, self.start_results, 1)
            self.robot[2].startSteps(self.hand3_com, self.start_results, 2)'''

            if (self.start_results[0][0] < 0) or (self.start_results[1][0] < 0) or (self.start_results[2][0] < 0):
                logging.error(
                    f'Error occurred on coordinates: ({shift0[i]}, {angle0[i]}), ({shift1[i]}, {angle1[i]}), ({shift2[i]}, {angle2[i]})\nAbort the move')
                break

    def calc_steps_and_ARR(self, a1, l1, a2, l2, a3, l3):
        t_get1 = threading.Thread(target=self.robot[0].get,
                                  args=(self.hand_com[0], self.get_results, 0, False))
        t_get2 = threading.Thread(target=self.robot[1].get,
                                  args=(self.hand_com[1], self.get_results, 1, False))
        t_get3 = threading.Thread(target=self.robot[2].get,
                                  args=(self.hand_com[2], self.get_results, 2, False))
        t_get1.start()
        t_get2.start()
        t_get3.start()

        t_get1.join()
        t_get2.join()
        t_get3.join()

        # TODO check if get results are ok

        # print(self.get_results)
        logging.error(
            f"BEFORE: ({self.get_results[0][4]:.3f}, {self.get_results[0][5]:.3f}), ({self.get_results[1][4]:.3f}, {self.get_results[1][5]:.3f}), ({self.get_results[2][4]:.3f}, {self.get_results[2][5]:.3f})")

        lin_1 = self.get_results[0][4]
        ang_1 = self.get_results[0][5]

        lin_2 = self.get_results[1][4]
        ang_2 = self.get_results[1][5]

        lin_3 = self.get_results[2][4]
        ang_3 = self.get_results[2][5]

        # pos_ang1 = abs(ang_1 - a1)
        # inverse_pos_ang1 = abs(360.0 - pos_ang1)
        # actualPosAngle1 = 0
        #
        # pos_ang2 = abs(ang_2 - a2)
        # inverse_pos_ang2 = abs(360.0 - pos_ang2)
        # actualPosAngle2 = 0
        #
        # pos_ang3 = abs(ang_3 - a3)
        # inverse_pos_ang3 = abs(360.0 - pos_ang3)
        # actualPosAngle3 = 0

        actualPosAngle1, actualPosDistance1, angDir1, linDir1 = self.get_pos_send_dir(0, lin_1, l1, ang_1, a1)
        actualPosAngle2, actualPosDistance2, angDir2, linDir2 = self.get_pos_send_dir(1, lin_2, l2, ang_2, a2)
        actualPosAngle3, actualPosDistance3, angDir3, linDir3 = self.get_pos_send_dir(2, lin_3, l3, ang_3, a3)

        motorStep = 200
        drvMicroSteps = 16
        steps4OneMM = 200 * drvMicroSteps / (2 * 20)

        anglePsteps1 = (actualPosAngle1 * (8 * motorStep * drvMicroSteps)) / 360
        distPsteps1 = actualPosDistance1 * steps4OneMM

        anglePsteps2 = (actualPosAngle2 * (8 * motorStep * drvMicroSteps)) / 360
        distPsteps2 = actualPosDistance2 * steps4OneMM

        anglePsteps3 = (actualPosAngle3 * (8 * motorStep * drvMicroSteps)) / 360
        distPsteps3 = actualPosDistance3 * steps4OneMM

        # period = 30
        period = 300

        steps_periods = [[distPsteps1, anglePsteps1, distPsteps2, anglePsteps2, distPsteps3, anglePsteps3],
                         [period, period, period, period, period, period]]

        max_steps = steps_periods[0][0]
        for i in range(6):
            if steps_periods[0][i] > max_steps:
                max_steps = steps_periods[0][i]

        for i in range(6):
            if steps_periods[0][i] != max_steps:
                delimiter = float(max_steps) / max(float(steps_periods[0][i]), 1)
                mnoj = ceil(period * delimiter)
                steps_periods[1][i] = mnoj

        steps_periods[0][0] *= linDir1
        steps_periods[0][1] *= angDir1
        steps_periods[0][2] *= linDir2
        steps_periods[0][3] *= angDir2
        steps_periods[0][4] *= linDir3
        steps_periods[0][5] *= angDir3

        '''logging.error(f"l1 from {lin_1} to {l1} in {distPsteps1} with {linDir1}, period={steps_periods[1][0]}") 
        logging.error(f"a1 from {ang_1} to {a1} in {anglePsteps1} with {angDir1}, period={steps_periods[1][1]}") 
        logging.error(f"l2 from {lin_2} to {l2} in {distPsteps2} with {linDir2}, period={steps_periods[1][2]}") 
        logging.error(f"a2 from {ang_2} to {a2} in {anglePsteps2} with {angDir2}, period={steps_periods[1][3]}") 
        logging.error(f"l3 from {lin_3} to {l3} in {distPsteps3} with {linDir3}, period={steps_periods[1][4]}") 
        logging.error(f"a3 from {ang_3} to {a3} in {anglePsteps3} with {angDir3}, period={steps_periods[1][5]}")'''

        return steps_periods

    def get_pos_send_dir(self, i, curr_lin, dest_lin, curr_ang, dest_ang):
        pos_ang = abs(curr_ang - dest_ang)
        inverse_pos_ang = abs(360.0 - pos_ang)
        actualPosAngle = 0
        actualPosDistance = abs(curr_lin - dest_lin)
        angDir = 0
        linDir = 0

        if inverse_pos_ang < pos_ang:
            actualPosAngle = inverse_pos_ang
            if curr_ang < dest_ang:
                # enable inverse
                angDir = -1
                # self.robot[i].setAngDir(1)
            elif curr_ang > dest_ang:
                # disable inverse
                angDir = 1
                # self.robot[i].setAngDir(0)
        else:
            actualPosAngle = pos_ang
            if curr_ang < dest_ang:
                # disable inverse
                angDir = 1
                # self.robot[i].setAngDir(0)
            elif curr_ang > dest_ang:
                # enable inverse
                angDir = -1
                # self.robot[i].setAngDir(1)

        if curr_lin < dest_lin:
            # enable inverse
            linDir = -1
            # self.robot[i].setLinDir(1)
        elif curr_lin > dest_lin:
            # disable inverse
            linDir = 1
            # self.robot[i].setLinDir(0)

        return actualPosAngle, actualPosDistance, angDir, linDir

