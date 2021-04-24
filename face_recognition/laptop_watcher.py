from collections import defaultdict
import time
from datetime import datetime, timedelta

import os

from watcher import Watcher
from detector import FaceDetector

def main():
    watcher = Watcher(
        url=0,
        detector=FaceDetector(),
        show_window=True,
        resize_factor=0.25
    )
    watcher.run()


if __name__ == '__main__':
    main()