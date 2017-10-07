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
import numpy as np

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

def main():
    print(__doc__)
    mainWindow = 'Charros vision'

    try:
        fn = sys.argv[1]
    except:
        fn = 1

    def nothing(*arg):
        pass

    cv2.namedWindow(mainWindow)
    #faceCascade = cv2.CascadeClassifier(cascPath)
    print get_script_path()
    face_cascade = cv2.CascadeClassifier(os.path.join(get_script_path(), 'haarcascade_frontalface_default.xml'))

    cap = cv2.VideoCapture(fn)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    width = cap.get(3)
    height = cap.get(4)
    scale = 2
    while True:
        flag, img_orig = cap.read()
        img = img_orig
        height, width = img.shape[:2]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (width/scale,height/scale))

        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(8, 8),
            flags = cv2.CASCADE_SCALE_IMAGE
        )

        vis_e = img.copy()
        roi_gray = None

        print "------"
        for (x, y, w, h) in faces:
            xs, ys, ws, hs = (int(x * scale), int(y * scale), int(w * scale), int(h * scale))
            print (xs, ys, ws, hs)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[ys:ys+hs, xs:xs+ws]
            overlay_image_alpha(vis_e,
                    roi_color[:, :, 0:3],
                    (0, 0),
                    np.ones(roi_color.shape[0:2]))

            print xs, ys, (xs + ws), (ys + hs)
            cv2.rectangle(vis_e, (xs, ys), ((xs + ws), (ys + hs)), (0, 255, 0), 2)



        cv2.imshow(mainWindow, vis_e)



        ch = cv2.waitKey(5) & 0xFF

        if ch == 27: # Esc
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
