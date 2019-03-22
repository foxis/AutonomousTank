#!/usr/local/bin/python3

from EasyVision.vision import *
from EasyVision.processors import *
from EasyVision.engine import *
import json
from EasyVision.processorstackbuilder import Args, Builder
import cv2


def main():
    with open("stereo_camera.json") as f:
        camera_model = StereoCamera.fromdict(json.load(f))

    builder = Builder(
        Builder(
            PyroCapture, Args('ExtractorLeft', nameserver="192.168.1.100"),
            CalibratedCamera, Args(None)
        ),
        Builder(
            PyroCapture, Args('ExtractorRight', nameserver="192.168.1.100"),
            CalibratedCamera, Args(None)
        ),
        CalibratedStereoCamera, Args(camera_model, display_results=True)
    )

    with builder.build() as vision:
        for frame in vision:
            if cv2.waitKey(1) == 27:
                break


if __name__ == "__main__":
    main()
