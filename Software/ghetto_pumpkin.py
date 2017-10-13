#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Ghetto pumpkin main logic

Created on Thu Oct 05 19:20:23 2017

@author: Tennessee Carmel-Veilleux
Copyright 2017, Tennessee Carmel-Veilleux
"""
from __future__ import print_function
import time
import socket
import argparse
import threading
from Queue import Queue, Empty


class Animator(object):
    """
    Basic animation thread with time-steps management
    """
    def __init__(self, update_rate, *args, **kwargs):
        self._config = kwargs.get("config", {})

        # Accumulated time
        self._t = 0.0
        self._init_time = 0.0

        self._last_update = 0.0
        self._update_rate = update_rate

        self._thread = threading.Thread(target=self._thread_process)
        self._queue = Queue()

    def start(self):
        if not self._thread.is_alive():
            self._thread.start()

    def stop(self, timeout=1.0):
        if self._thread.is_alive():
            self._queue.put({"event": "shutdown"})
            self._thread.join(timeout=timeout)

    def _setup(self):
        pass

    def _teardown(self):
        pass

    def _update(self):
        pass

    def _handle_event(self, event_type, event_data):
        pass

    def queue_event(self, event_type, event_data):
        self._queue.put({"event": event_type, "data": event_data})

    def _thread_process(self):
        self._init_time = time.time()
        self._last_update = self._init_time
        self._t = 0.0
        self._setup()

        while True:
            # Process time steps to run update
            t = time.time()
            elapsed = t - self._last_update
            self._t = t - self._init_time

            if elapsed >= self._update_rate:
                self._last_update = t
                self._update()
                event_timeout = 0.0
            else:
                event_timeout = self._update_rate - elapsed

            # Handle events coming into event loop
            try:
                event = self._queue.get(block=True, timeout=event_timeout)
                event_type = event.get("event", "")

                if event_type == "shutdown":
                    break
                else:
                    event_data = event.get("data", None)
                    self._handle_event(event_type, event_data)
            except Empty:
                continue

        self._teardown()


class PeriodicAnimator(Animator):
    def __init__(self, update_rate, *args, **kwargs):
        super(PeriodicAnimator, self).__init__(update_rate, *args, **kwargs)

    def _setup(self):
        print("_setup() called")

    def _update(self):
        print("_update() called, t=%.3f" % self._t)

    def _teardown(self):
        print("_teardown() called")

    def _handle_event(self, event_type, event_data):
        print("_handle_event called, event_type=%s, event_data=%s" % (event_type, event_data))


class LedPWMAnimator(Animator):
    ALGO_BACK_AND_FORTH = 1

    def __init__(self, rgb_led_setter, **kwargs):
        super(LedPWMAnimator, self).__init__(0.1)
        self._rgb_led_setter = rgb_led_setter
        self._back_and_forth_time = kwargs.get("back_and_forth_time", 2.0)
        self._back_and_forth_start = kwargs.get("back_and_forth_start", (0.0, 0.0, 0.0))
        self._back_and_forth_end = kwargs.get("back_and_forth_end", (1.0, 1.0, 1.0))
        self._algorithm = self.ALGO_BACK_AND_FORTH
        self._last_t = 0.0
        self._cycle_start = 0.0
        self._direction = 1

    def _setup(self):
        self._rgb_led_setter(*self._back_and_forth_start)
        self._last_t = self._t
        self._cycle_start = self._last_t

    def _update_back_and_forth(self):
        cycle_t = (self._t - self._cycle_start) / self._back_and_forth_time
        if cycle_t >= 1.0:
            cycle_t = 0.0
            self._direction = -self._direction
            self._cycle_start = self._t

        if self._direction > 0:
            sr, sg, sb = self._back_and_forth_start
            er, eg, eb = self._back_and_forth_end
        else:
            sr, sg, sb = self._back_and_forth_end
            er, eg, eb = self._back_and_forth_start

        r = sr + (cycle_t * (er - sr))
        g = sg + (cycle_t * (eg - sg))
        b = sb + (cycle_t * (eb - sb))

        self._rgb_led_setter(r, g, b)

    def _update(self):
        if self._algorithm == self.ALGO_BACK_AND_FORTH:
            self._update_back_and_forth()

    def _teardown(self):
        pass

    def _handle_event(self, event_type, event_data):
        pass


class MaestroServoPulseSetter(object):
    def __init__(self, serial_device):
        from drivers.pololu_maestro import PololuMaestro
        self._serial_device = serial_device
        self._driver = PololuMaestro(serial_device)
        self._driver.connect()
        self._lock = threading.Lock()
        self._is_open = True

    def get_channel_setter(self, channel_id):
        def setter(microseconds):
            counts = int(microseconds * 4.0)
            with self._lock:
                if not self._is_open:
                    return
                self._driver.set_target(channel_id, counts)

        return setter

    def shutdown(self):
        with self._lock:
            self._is_open = False
            self._driver.close()


def sign(val):
    if val < 0:
        return -1
    else:
        return 1


class EyeController(Animator):
    """
    Animation controller for the position of a servo-controlled eye.

    Config:
    * min_us = minimum servo pulse duration for one edge [microseconds]
    * center_us = servo pulse duration for center of eye (in case min/max are not symmetrical from center)  [microseconds]
    * max_us = maximum servo pulse duration for other edge [microseconds]
    * eye_speed = number of seconds taken to travel from one side of the eye to the other [seconds]
    """
    STATE_BACK_AND_FORTH = "back_and_forth"
    STATE_HOLD = "hold"

    EVENT_EYE_POSITION = "eye_position"
    EVENT_RESUME_BACK_AND_FORTH = "resume_back_and_forth"

    def __init__(self, servo_setter, *args, **kwargs):
        super(EyeController, self).__init__(update_rate=kwargs.get("update_rate", 0.1))
        self._min_us = kwargs.get("min_us", 1500.0)
        self._max_us = kwargs.get("max_us", 1500.0)
        self._center_us = kwargs.get("center_us", 1500.0)
        # Eye speed is in "seconds per full side-to-side motion"
        self._eye_speed = kwargs.get("eye_speed", 10.0)
        self._verbose = kwargs.get("verbose", False)
        self._servo_setter = servo_setter

        self._state = self.STATE_BACK_AND_FORTH
        self._last_t = 0.0
        self._direction = 1
        self._current_us = self._center_us
        self._target_us = self._max_us

    def _set_microseconds(self, microseconds):
        microseconds = max(self._min_us, microseconds)
        microseconds = min(self._max_us, microseconds)
        self._servo_setter(microseconds)

    def _set_state(self, new_state):
        # Optional logging hook
        if new_state != self._state:
            if self._verbose:
                print("%s: %s -> %s" % (self.__class__.__name__, self._state, new_state))
            self._state = new_state

    def _setup(self):
        self._set_microseconds(self._current_us)
        self._last_t = self._t

    def _update(self):
        delta = self._t - self._last_t
        self._last_t = self._t

        distance_us = (delta / self._eye_speed) * (self._max_us - self._min_us)
        error_us = self._current_us - self._target_us
        if self._verbose:
            print("current_us=%d, target_us=%d, error_us=%d, distance_us=%d" % (int(self._current_us), int(self._target_us), int(error_us), int(distance_us)))

        if 0 == error_us:
            # Target reached!
            if self._state == self.STATE_BACK_AND_FORTH:
                # Back and forth: change direction by setting target to other extreme
                self._direction = -self._direction
                if self._direction < 0:
                    self._target_us = self._min_us
                else:
                    self._target_us = self._max_us
            elif self._state == self.STATE_HOLD:
                # Hold mode: don't move, we're there !
                pass
            return
        elif abs(error_us) < distance_us:
            # About to reach target, go to target in this interval
            self._current_us = self._target_us
        else:
            # Far from target: move by how much speed allows
            self._current_us -= sign(error_us) * distance_us

        self._servo_setter(self._current_us)

    def _teardown(self):
        pass

    def _handle_event(self, event_type, event_data):
        if event_type == self.EVENT_EYE_POSITION:
            # Force eye to move to position, using ratio of position within eye, on or the other side from the middle
            eye_position = event_data
            if eye_position < 0.5:
                ratio = ((0.5 - eye_position) / 0.5)
                self._target_us = self._center_us - (ratio * (self._center_us - self._min_us))
            else:
                ratio = ((eye_position - 0.5) / 0.5)
                self._target_us = self._center_us + (ratio * (self._max_us - self._center_us))
            self._set_state(self.STATE_HOLD)

        elif event_type == self.EVENT_RESUME_BACK_AND_FORTH:
            # Set target to max extent of direction we had previously been going
            if self._direction < 0:
                self._target_us = self._min_us
            else:
                self._target_us = self._max_us

            self._set_state(self.STATE_BACK_AND_FORTH)

    def set_eye_position(self, position):
        """
        Set eye to go to a given position and hold

        :param position: normalized position: 0.0 = min_us, 0.5 = middle_us, 1.0 = max_us
        """
        self.queue_event(self.EVENT_EYE_POSITION, position)

    def resume_back_and_forth(self):
        self.queue_event(self.EVENT_RESUME_BACK_AND_FORTH, None)

class PumpkinStateMachine(object):
    def __init__(self):
        self.event_queue = Queue()

def rgb_led_setter(r, g, b):
    print("r=%.2f, g=%.2f, b=%.2f" % (r, g, b))

def dummy_servo_setter(microseconds):
    print("About to set servo target %dus" % int(microseconds))


if __name__ == '__main__':
    if True:
        eye_config = {
            "min_us": 1295.0,
            "max_us": 1828.0,
            "center_us": 1535.0,
            "eye_speed": 3.0
        }
        serial_device = "COM37"

        if False:
            maestro = MaestroServoPulseSetter(serial_device)
            eyes_servo_setter = maestro.get_channel_setter(0)
        else:
            eyes_servo_setter = dummy_servo_setter

        eye_animator = EyeController(eyes_servo_setter, verbose=True, **eye_config)
        eye_animator.start()
        time.sleep(2.0)
        eye_animator.set_eye_position(0.25)
        time.sleep(5.0)
        eye_animator.resume_back_and_forth()
        time.sleep(2.0)
        eye_animator.stop()

    if False:
        led_animator = LedPWMAnimator(rgb_led_setter)
        led_animator.start()
        time.sleep(2.0)
        led_animator.queue_event("set_value", 10.0)
        time.sleep(3.0)
        led_animator.stop()

    if False:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = "127.0.0.1"
        port = 5005
        sock.bind((addr, port))
        sock.settimeout(0.5)

        while True:
            data, addr = sock.recvfrom(2048)
            print("From %s: %s" % (addr, data))
