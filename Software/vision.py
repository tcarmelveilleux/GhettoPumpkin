#!/usr/bin/env python2
'''
Vision for robot.
---
    Charros vision
    Copyright (C) 2017 Matthew Bells

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import cv2
import os
import os.path
import sys
import collections
import json
import socket
import argparse
import numpy as np

DEFAULT_UDP_PORT = 6321
DEFAULT_UDP_ADDR = "127.0.0.1"
DEFAULT_WIDTH = 320
DEFAULT_HEIGHT = 240
DEFAULT_DOWNSCALE = 2

def overlay_image_alpha(img, img_overlay, pos, alpha_mask):
    """Overlay img_overlay on top of img at the position specified by
    pos and blend using alpha_mask.

    Alpha mask must contain values within the range [0, 1] and be the
    same size as img_overlay.

    From: https://stackoverflow.com/a/45118011
    """

    x, y = pos

    # Image ranges
    y1, y2 = max(0, y), min(img.shape[0], y + img_overlay.shape[0])
    x1, x2 = max(0, x), min(img.shape[1], x + img_overlay.shape[1])

    # Overlay ranges
    y1o, y2o = max(0, -y), min(img_overlay.shape[0], img.shape[0] - y)
    x1o, x2o = max(0, -x), min(img_overlay.shape[1], img.shape[1] - x)

    # Exit if nothing to do
    if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
        return

    channels = img.shape[2]

    alpha = alpha_mask[y1o:y2o, x1o:x2o]
    alpha_inv = 1.0 - alpha

    for c in range(channels):
        img[y1:y2, x1:x2, c] = (alpha * img_overlay[y1o:y2o, x1o:x2o, c] +
                                alpha_inv * img[y1:y2, x1:x2, c])

def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))

class FaceDetector(object):
    def __init__(self, cap_device, scale_down=2, *args, **kwargs):
        self._cap_device = cap_device
        self._width = cap_device.get(3)
        self._height = cap_device.get(4)
        self._scale_down = scale_down
        self._face_cascade = cv2.CascadeClassifier(os.path.join(get_script_path(), 'haarcascade_frontalface_default.xml'))

    def emit_faces(self, faces):
        pass

    def do_one_frame(self, output_window=None):
        flag, img_orig = self._cap_device.read()
        img = img_orig
        height, width = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (width / self._scale_down, height / self._scale_down))

        faces_detected = self._face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(8, 8),
            flags=cv2.CASCADE_SCALE_IMAGE
        )

        vis_e = img.copy()

        faces = []
        for (x, y, w, h) in faces_detected:
            scale = self._scale_down
            xs, ys, ws, hs = (int(x * scale), int(y * scale), int(w * scale), int(h * scale))
            #roi_gray = gray[y:y + h, x:x + w]
            #roi_color = img[ys:ys + hs, xs:xs + ws]
            #overlay_image_alpha(vis_e,
            #                    roi_color[:, :, 0:3],
            #                    (0, 0),
            #                    np.ones(roi_color.shape[0:2]))

            #print xs, ys, (xs + ws), (ys + hs)
            cv2.rectangle(vis_e, (xs, ys), ((xs + ws), (ys + hs)), (0, 255, 0), 2)
            faces.append({"x": float(xs) / width, "y": float(ys) / height,
                          "width": float(ws) / width, "height": float(hs) / height})

        if output_window is not None:
            cv2.imshow(output_window, vis_e)

        if len(faces) > 0:
            self.emit_faces(faces)

class UDPFaceDetector(FaceDetector):
    def __init__(self, addr, port, cap_device, scale_down=2, *args, **kwargs):
        super(UDPFaceDetector, self).__init__(cap_device, scale_down, *args, **kwargs)
        self._addr = addr
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = 0

    def emit_faces(self, faces):
        cooked_faces = []
        for face in faces:
            x = face["x"]
            y = face["y"]
            width = face["width"]
            height = face["height"]

            cooked_face = collections.OrderedDict()
            cooked_face["x"] = x
            cooked_face["y"] = y
            cooked_face["cx"] = x + (width / 2.0)
            cooked_face["cy"] = y + (height / 2.0)
            cooked_face["width"] = width
            cooked_face["height"] = height
            cooked_face["area"] = width * height

            cooked_faces.append(cooked_face)

        # Sort faces by descending area
        cooked_faces = sorted(cooked_faces, reverse=True, key=lambda face: face["area"])

        # Emit faces array as JSON
        faces_json = json.dumps({"faces": cooked_faces, "seq": self._seq}, indent=2)
        self._seq += 1

        # Send it to the socket
        self._sock.sendto(faces_json, (self._addr, self._port))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', metavar="UDP_PORT", default=DEFAULT_UDP_PORT, type=int, action="store",
                        help='Set UDP port for output (default: %d)' % DEFAULT_UDP_PORT)
    parser.add_argument('-a', '--address', metavar="UDP_ADDR", default=DEFAULT_UDP_ADDR, action="store",
                        help='Set UDP address for output (default: %s)' % DEFAULT_UDP_ADDR)
    parser.add_argument('-c', '--camera', metavar="CAMERA_NUM", action="store", default=0, type=int, help='Set camera number to use (default: 0)')
    parser.add_argument('-W', '--width', metavar="WIDTH_PIX", action="store", type=int,
                        default=DEFAULT_WIDTH, help='Set camera image width in pixels (default:%d)' % DEFAULT_WIDTH)
    parser.add_argument('-H', '--height', metavar="HEIGHT_PIX", action="store", type=float,
                        default=DEFAULT_WIDTH, help='Set camera image height in pixels (default:%d)' % DEFAULT_HEIGHT)
    parser.add_argument('-d', '--downscale', metavar="FACTOR", action="store", type=int,
                        default=DEFAULT_DOWNSCALE, help='Set downscaling integer factor (default:%d)' % DEFAULT_DOWNSCALE)
    parser.add_argument('-v', '--view', action="store_true", default=False,
                        help='Enable viewing of camera and faces')

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    if args.view:
        main_window = 'facedetector'
        cv2.namedWindow(main_window)
    else:
        main_window = None

    cam_idx = args.camera
    cap = cv2.VideoCapture(cam_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    face_detector = UDPFaceDetector(args.address, args.port, cap, scale_down=args.downscale)

    while True:
        face_detector.do_one_frame(output_window=main_window)

        try:
            if main_window is not None:
                ch = cv2.waitKey(5) & 0xFF

                if ch == 27: # Esc
                    break
        except KeyboardInterrupt:
            break

    if main_window is not None:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
