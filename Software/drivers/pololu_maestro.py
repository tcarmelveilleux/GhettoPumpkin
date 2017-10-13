# -*- coding: utf-8 -*-
#
# Basic library to interface with Pololu Maestro servo controllers
# through serial port.
#
# Limitations:
# * CRC-7 not yet supported
#
# Copyright 2016-2017, Tennessee Carmel-Veilleux

import serial

class PololuMaestro:
    CMD_SET_TARGET = 0x04
    CMD_SET_SPEED = 0x07
    CMD_SET_ACCEL = 0x09

    def __init__(self, port, baud_rate=9600):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_port = None

    def connect(self):
        if self.serial_port != None:
            self.close()

        self.serial_port = serial.Serial(self.port, self.baud_rate)

        return True

    def close(self):
        if self.serial_port != None:
            self.serial_port.close()
            self.serial_port = None

    def set_target(self, servo_id, target_counts, device_number = 12):
        bytes_to_send = bytearray([0xaa, device_number & 0x7f, self.CMD_SET_TARGET])
        bytes_to_send.append(servo_id & 0x1f)
        bytes_to_send.append(target_counts & 0x7f)
        bytes_to_send.append((target_counts >> 7) & 0x7f)
        #print ", ".join(["0x%02X" % b for b in bytes_to_send])
        self.serial_port.write(bytes_to_send)

    def set_speed(self, servo_id, speed, device_number = 12):
        bytes_to_send = bytearray([0xaa, device_number & 0x7f, self.CMD_SET_SPEED])
        bytes_to_send.append(servo_id & 0x1f)
        bytes_to_send.append(speed & 0x7f)
        bytes_to_send.append((speed >> 7) & 0x7f)
        self.serial_port.write(bytes_to_send)

    def set_accel(self, servo_id, acceleration, device_number = 12):
        bytes_to_send = bytearray([0xaa, device_number & 0x7f, self.CMD_SET_ACCEL])
        bytes_to_send.append(servo_id & 0x1f)
        bytes_to_send.append(acceleration & 0x7f)
        bytes_to_send.append((acceleration >> 7) & 0x01)
        self.serial_port.write(bytes_to_send)
