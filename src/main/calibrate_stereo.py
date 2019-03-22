# -*- coding: utf-8 -*-
"""Stereo camera calibration tool.
Will calibrate a stereo camera pair using 9x7 rectangular calibration pattern.::

    usage: calibrate_stereo.py [-h] [-f FILE] [-i SIZE] [-p FPS] [-g GRID] [-N N]
                               [-t] [-d]
                               left right

    Stereo Camera calibration tool

    positional arguments:
      left                  Left Camera device ID/folder
      right                 Left Camera device ID/folder

    optional arguments:
      -h, --help            show this help message and exit
      -f FILE, --file FILE  Output filename of the calibrated camera
      -i SIZE, --size SIZE  Frame width and height
      -p FPS, --fps FPS     Frame rate
      -g GRID, --grid GRID  Grid shape of the calibration target
      -N N                  Number of samples to gather
      -t, --test            Test camera calibration file
      -d, --disparity       Calculate Disparity Map

"""

from argparse import ArgumentParser
from EasyVision.vision import PyroCapture
from EasyVision.processors import CalibratedStereoCamera, StereoCamera
import json
import cv2


def main():
    parser = ArgumentParser(description="Stereo Camera calibration tool")
    parser.add_argument("-f", "--file", default="stereo_camera.json", help="Output filename of the calibrated camera")
    parser.add_argument("-i", "--size", default="640,480", help="Frame width and height")
    parser.add_argument("-p", "--fps", default="5", help="Frame rate")
    parser.add_argument("-g", "--grid", default="9,7", help="Grid shape of the calibration target")
    parser.add_argument("-N", type=int, default=30, help="Number of samples to gather")
    parser.add_argument("-t", "--test", const=True, default=False, action='store_const',
                        help="Test camera calibration file")
    parser.add_argument("-d", "--disparity", const=True, default=False, action='store_const',
                        help="Calculate Disparity Map")

    args = parser.parse_args()
    grid = tuple(int(i) for i in args.grid.split(','))
    size = tuple(int(i) for i in args.size.split(','))

    camera_model = None
    if args.test:
        with open(args.file) as f:
            camera_model = StereoCamera.fromdict(json.load(f))
            print("{} was loaded for evaluation".format(args.file))

    left = PyroCapture('ExtractorLeft', nameserver="192.168.1.100")
    right = PyroCapture('ExtractorRight', nameserver="192.168.1.100")

    camera = CalibratedStereoCamera(left, right, camera_model, display_results=True)

    if args.test:
        with camera as vision:
            for frame in vision:
                if cv2.waitKey(1) == 27:
                    break
    else:
        with camera as vision:
            cam = None
            while not cam:
                cam = vision.calibrate()
                if cv2.waitKey(1) == 27:
                    print("Calibration was aborted")
                    return

            with open(args.file, "w") as f:
                f.write(json.dumps(cam.todict(), indent=4))
                print("{} was written".format(args.file))


if __name__ == "__main__":
    main()
