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
import json
import socket
import argparse
import threading
import subprocess
from Queue import Queue, Empty

DEFAULT_UDP_PORT = 6321
DEFAULT_UDP_ADDR = "127.0.0.1"

class Animator(object):
    """
    Basic animation thread with time-steps management
    """
    def __init__(self, id, update_rate, *args, **kwargs):
        self._config = kwargs.get("config", {})
        self._id = id

        # Accumulated time
        self._t = 0.0
        self._init_time = 0.0

        self._last_update = 0.0
        self._update_rate = update_rate

        self._thread = threading.Thread(target=self._thread_process)
        self._queue = Queue()

    @property
    def id(self):
        return self._id

    def start(self):
        if not self._thread.is_alive():
            self._thread.start()

    def shutdown(self, timeout=1.0):
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
    def __init__(self, id, update_rate, *args, **kwargs):
        super(PeriodicAnimator, self).__init__(id, update_rate, *args, **kwargs)

    def _setup(self):
        print("_setup() called")

    def _update(self):
        print("_update() called, t=%.3f" % self._t)

    def _teardown(self):
        print("_teardown() called")

    def _handle_event(self, event_type, event_data):
        print("_handle_event called, event_type=%s, event_data=%s" % (event_type, event_data))


class LedPWMAnimator(Animator):
    ALGO_BACK_AND_FORTH = "back_and_forth"

    def __init__(self, id, rgb_led_setter, **kwargs):
        super(LedPWMAnimator, self).__init__(id, 0.1)
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

    def __init__(self, id, servo_setter, *args, **kwargs):
        super(EyeController, self).__init__(id=id, update_rate=kwargs.get("update_rate", 0.05))
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


class Driver(object):
    """
    Base class for drivers. Each driver has its own extended interface on top of these basic methods.
    """
    def __init__(self, id="driver", **kwargs):
        self._lock = threading.Lock()
        self._id = id

    @property
    def id(self):
        return self._id

    def start(self):
        pass

    def shutdown(self):
        pass

    def get_channel(self, channel_id):
        """
        Return a setter callable for the given internal channel, or a driver-specific
        instance that allows the channel to be operated. The returned object must be
        valid until `shutdown()` is called.

        :param channel_id: channel identifier (usually an integer)
        :return: object to operate the given channel
        """
        return None


class ServoDriver(Driver):
    def __init__(self, id="servo", **kwargs):
        super(ServoDriver, self).__init__(id=id, **kwargs)

    def get_channel(self, channel_id):
        def setter(microseconds):
            # channel_id can be closured here
            pass
        return setter


class MaestroServoDriver(ServoDriver):
    def __init__(self, **kwargs):
        super(MaestroServoDriver, self).__init__(**kwargs)
        self._serial_device = kwargs["serial_device"]
        self._is_open = False
        self._driver = None

    def start(self):
        with self._lock:
            from drivers.pololu_maestro import PololuMaestro
            self._driver = PololuMaestro(self._serial_device)
            self._driver.connect()
            self._is_open = True

    def get_channel(self, channel_id):
        def setter(microseconds):
            counts = int(microseconds * 4.0)
            with self._lock:
                if self._driver is None or not self._is_open:
                    return
                self._driver.set_target(channel_id, counts)
        return setter

    def shutdown(self):
        with self._lock:
            self._is_open = False
            self._driver.close()


class ConsoleServoDriver(ServoDriver):
    def __init__(self, **kwargs):
        super(ConsoleServoDriver, self).__init__(**kwargs)

    def get_channel(self, channel_id):
        def setter(microseconds):
            with self._lock:
                print("Setting servo id=%s channel %d to %dus" % (self._id, channel_id, microseconds))
        return setter


class RgbLedDriver(Driver):
    def __init__(self, **kwargs):
        super(RgbLedDriver, self).__init__(**kwargs)

    def get_channel(self, channel_id):
        def setter(r, g, b):
            # Set the RGB to normalized R [0..1], G [0..1], B [0..1]. Can closure the channel_id
            pass
        return setter


class ConsoleRgbLedDriver(RgbLedDriver):
    def __init__(self, **kwargs):
        super(ConsoleRgbLedDriver, self).__init__(**kwargs)
        self._quiet = kwargs.get("quiet", False)

    def get_channel(self, channel_id):
        def setter(r, g, b):
            if self._quiet: return
            with self._lock:
                print("Setting RGB driver id=%s channel %d to (%.3f, %.3f, %.3f)" % (self._id, channel_id, r, g, b))
        return setter


def sign(val):
    if val < 0:
        return -1
    else:
        return 1


class VisionProcess(Driver):
    """
    Basic animation thread with time-steps management
    """
    def __init__(self, id, **kwargs):
        super(VisionProcess, self).__init__(id=id, **kwargs)
        self._id = id

        self._udp_addr = kwargs["udp_addr"]
        self._udp_port = kwargs["udp_port"]
        self._camera_idx = kwargs["camera_idx"]
        self._process_cmd = kwargs["process_cmd"]
        self._view_image = kwargs["view_image"]
        self._process = None
        self._handler = None
        self._socket = None
        self._thread = threading.Thread(target=self._thread_process)
        self._queue = Queue()

    @property
    def id(self):
        return self._id

    def set_handler(self, handler):
        self._queue.put({"event": "set_handler", "handler": handler})

    def start(self):
        if not self._thread.is_alive():
            self._thread.start()

    def shutdown(self, timeout=1.0):
        if self._thread.is_alive():
            self._queue.put({"event": "shutdown"})
            self._thread.join(timeout=timeout)

    def _launch_process(self):
        full_cmd = "%s -c %d -a %s -p %d --mirror %s" % (self._process_cmd, self._camera_idx,
                                                         self._udp_addr, self._udp_port,
                                                         "-v" if self._view_image else "")
        self._process = subprocess.Popen(full_cmd)

    def _thread_process(self):
        self._launch_process()

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((self._udp_addr, self._udp_port))
        self._socket.settimeout(0.2)

        done = False
        while not done:
            # Drain our event queue
            while True:
                try:
                    event = self._queue.get(block=False)
                    if event["event"] == "set_handler":
                        self._handler = event["handler"]
                    elif event["event"] == "shutdown":
                        done = True
                except Empty:
                    break

            # Try to receive a UDP frame from the vision process for a while
            try:
                data, addr = self._socket.recvfrom(2048)
                vision_event = json.loads(data)
                if self._handler is not None:
                    self._handler(vision_event)

            except socket.timeout:
                pass

        # Done: terminate vision server
        try:
            self._process.terminate()
            self._process.wait()
        except subprocess.CalledProcessError:
            pass


class PumpkinAnimator(Animator):
    """
    Main Animation for the Ghetto Pumpkin.

    Glues together all the drivers and animators to make the pumpkin follow a face
    target if one is available, or otherwise move its eyes back and forth with a
    color change taking effect.
    """
    STATE_BACK_AND_FORTH = "back_and_forth"
    STATE_FIXATED = "fixated"

    def __init__(self, drivers, mappings, eyes_controller, left_led_controller, right_led_controller):
        super(PumpkinAnimator, self).__init__("pumpkin_animator", 0.1)
        self._drivers = drivers
        self._mappings = mappings
        self._eyes_controller = eyes_controller
        self._left_led_controller = left_led_controller
        self._right_led_controller = right_led_controller
        self._fixation_timeout = 3.0
        self._verbose = True
        self._last_face_time = 0.0
        self._target_location = 0.5
        self._state = None

    def _setup(self):
        for id, driver in self._drivers.items():
            print("Starting driver: %s" % id)
            driver.start()

        for controller in [self._eyes_controller, self._left_led_controller, self._right_led_controller]:
            print("Starting controller: %s" % controller.id)
            controller.start()

        self._state = self.STATE_BACK_AND_FORTH
        self._last_face_time = self._t

    def _teardown(self):
        for controller in [self._eyes_controller, self._left_led_controller, self._right_led_controller]:
            print("Stopping controller: %s" % controller.id)
            controller.shutdown()

        for id, driver in self._drivers.items():
            print("Stopping driver: %s" % id)
            driver.shutdown()

    def _set_state(self, new_state):
        # Optional logging hook
        if new_state != self._state:
            if self._verbose:
                print("%s: %s -> %s" % (self.__class__.__name__, self._state, new_state))
            self._state = new_state

    def _update(self):
        fixation_expired = (self._t - self._last_face_time) > self._fixation_timeout
        if fixation_expired and self._state == self.STATE_FIXATED:
            self._eyes_controller.resume_back_and_forth()
            self._set_state(self.STATE_BACK_AND_FORTH)

    def _handle_event(self, event_type, event_data):
        if event_type == "face_detector":
            faces = event_data["faces"]
            if len(faces) == 0:
                return

            biggest_face = faces[0]
            self._target_location = biggest_face["cx"]
            print("Got face, location=%.3f" % self._target_location)
            self._last_face_time = self._t
            self._eyes_controller.set_eye_position(self._target_location)
            self._set_state(self.STATE_FIXATED)

    def handle_face_event(self, vision_event):
        self.queue_event("face_detector", vision_event)

def load_json_config(filename):
    with open(filename, "rb") as infile:
        return json.load(infile)


def create_driver(driver):
    driver_type = driver["type"]
    if driver_type == "maestro":
        driver_instance = MaestroServoDriver(**driver)
    elif driver_type == "servo_console":
        driver_instance = ConsoleServoDriver(**driver)
    elif driver_type == "rgb_led_console":
        driver_instance = ConsoleRgbLedDriver(**driver)
    elif driver_type == "vision_process":
        driver_instance = VisionProcess(**driver)
    else:
        raise KeyError("Don't know how to instantiate driver type '%s'" % driver_type)

    mappings = {}
    for name, channel_id in driver.get("mapping", {}).items():
        mappings[name] = driver_instance.get_channel(channel_id)

    return driver_instance, mappings


def instantiate_drivers(driver_config):
    mappings = {}
    drivers = {}

    for driver in driver_config["drivers"]:
        driver_id = driver["id"]
        driver_instance, driver_mappings = create_driver(driver)

        drivers[driver_id] = driver_instance
        mappings.update(driver_mappings)

    return drivers, mappings


def instantiate_config(driver_config, animation_config, profile="default"):
    profiles = animation_config["profiles"]

    # Generic drivers and channels mapping config instantiation
    drivers, mappings = instantiate_drivers(driver_config)

    # Ghetto-pumpkin-specific loading of animation params
    animation_profile = profiles.get(profile)

    eye_controller_profile = animation_profile["eye_controller"]
    eye_controller_driver = mappings[eye_controller_profile["mapping"]]
    eye_controller = EyeController(id="eye_controller", servo_setter=eye_controller_driver, **eye_controller_profile)

    left_eye_led_profile = animation_profile["left_eye_led"]
    left_eye_led_driver = mappings[left_eye_led_profile["mapping"]]
    left_eye_led_controller = LedPWMAnimator(id="left_eye_led", rgb_led_setter=left_eye_led_driver, **left_eye_led_profile)

    right_eye_led_profile = animation_profile["right_eye_led"]
    right_eye_led_driver = mappings[right_eye_led_profile["mapping"]]
    right_eye_led_controller = LedPWMAnimator(id="right_eye_led", rgb_led_setter=right_eye_led_driver, **right_eye_led_profile)

    face_detector = drivers["face_detector"]
    pumpkin_animator = PumpkinAnimator(drivers, mappings, eye_controller, left_eye_led_controller, right_eye_led_controller)
    face_detector.set_handler(pumpkin_animator.handle_face_event)

    return pumpkin_animator


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--animation-config', metavar="JSON_FILENAME", required=True, action="store",
                        help='Set config file to use for animation')
    parser.add_argument('-d', '--driver-config', metavar="JSON_FILENAME", required=True, action="store",
                        help='Set config file to use for drivers')

    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_args()

    # TODO: Add try/except around config load
    animation_config = load_json_config(args.animation_config)
    driver_config = load_json_config(args.driver_config)

    animation_setup = instantiate_config(driver_config, animation_config)
    animation_setup.start()
    time.sleep(60.0)
    animation_setup.shutdown()