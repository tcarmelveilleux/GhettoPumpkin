# -*- coding: utf-8 -*-
"""
Ghetto pumpkin main logic

Created on Thu Oct 05 19:20:23 2017

@author: Tennessee
"""
from __future__ import print_function
import time
import socket
import argparse
import threading
from Queue import Queue, Empty

class Animator(object):
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

    def _handle_event(self, event):
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
                if event.get("event", "") == "shutdown":
                    break
                else:
                    self._handle_event(event)
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

    def _handle_event(self, event):
        print("_handle_event called, event=%s" % event)


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

    def _handle_event(self, event):
        pass

class PumpkinStateMachine(object):
    def __init__(self):
        self.event_queue = Queue()

def rgb_led_setter(r, g, b):
    print("r=%.2f, g=%.2f, b=%.2f" % (r, g, b))

if __name__ == '__main__':
    #animator = PeriodicAnimator(update_rate=0.05)
    animator = LedPWMAnimator(rgb_led_setter)

    if False:
        animator.start()
        time.sleep(2.0)
        animator.queue_event("set_value", 10.0)
        time.sleep(3.0)
        animator.stop()
    else:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = "127.0.0.1"
        port = 5005
        sock.bind((addr, port))
        sock.settimeout(0.5)

        while True:
            data, addr = sock.recvfrom(2048)
            print("From %s: %s" % (addr, data))

