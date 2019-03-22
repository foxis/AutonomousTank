#!/usr/local/bin/python3

from EasyVision.vision import *
from EasyVision.processors import *
from EasyVision.engine import *
import json
import cv2


def main():
    with open("stereo_camera.json") as f:
        camera_model = StereoCamera.fromdict(json.load(f))

    left = PyroCapture('ExtractorLeft', nameserver="192.168.1.100")
    right = PyroCapture('ExtractorRight', nameserver="192.168.1.100")

    camera = CalibratedStereoCamera(left, right, camera_model, display_results=True)

    with camera as vision:
        for frame in vision:
            if cv2.waitKey(1) == 27:
                break


if __name__ == "__main__":
    main()
