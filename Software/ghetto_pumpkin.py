# -*- coding: utf-8 -*-
"""
Ghetto pumpkin main logic

Created on Thu Oct 05 19:20:23 2017

@author: Tennessee
"""

import time
import argparse
import threading
from Queue import Queue, Empty

class LedPulseStateMachine(object):
    ALGO_BACK_AND_FORTH = 1

    def __init__(self, **kwargs):
        self._red_led_handler = kwargs.get("red_led_hander")
        self._green_led_handler = kwargs.get("green_led_hander")
        self._blue_led_handler = kwargs.get("blue_led_hander")
        self._back_and_forth_time = kwargs.get("back_and_forth_time", 2.0)
        self._back_and_forth_start = kwargs.get("back_and_forth_start", (0.0, 0.0, 0.0))
        self._back_and_forth_end = kwargs.get("back_and_forth_end", (1.0, 0.0, 0.0))
        self._algorithm = self.ALGO_BACK_AND_FORTH

        # Accumulated time
        self._t = 0.0
        self._start_time = 0.0
        self._update_time = 0.01

        self._thread = threading.Thread(target=self._process)
        self._alive = False
        self._queue = Queue()

    def _process(self):
        self._alive = True

        while True:
            try:
                event = self._queue.get(block=True, timeout=self._update_time)
            except Empty:
                pass

            self = time.time() - self._start_time

class PumpkinStateMachine(object):
    def __init__(self):
        self.event_queue = Queue()

