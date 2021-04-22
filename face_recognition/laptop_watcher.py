from collections import defaultdict
import time
from datetime import datetime, timedelta

import os

from webcam import Webcam
from detector import FaceDetector

def main():
    webcam = Webcam(
        url=0,
        detector=FaceDetector(),
        show_window=True,
        resize_factor=0.25
    )
    webcam.run()


if __name__ == '__main__':
    main()