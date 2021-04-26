import cv2
from time import sleep
from log import logging
from detector import FaceDetector, load_img_2_rgb
from displayer import draw_locations, show_to_window

class Watcher:

    def __init__(self, url, detector, draw_locations=True, show_window=False, event_func=None, resize_factor=None):
        self.url = url
        self.detector = detector
        self.draw_locations = draw_locations
        self.show_window = show_window
        self.event_func = event_func
        self.resize_factor = resize_factor

        self.is_cam = self.url.isnumeric() or self.url.startswith('http')

        self.cap = cv2.VideoCapture(url)

    def run(self):
        logging.info(f'Start detect from webcam: {self.url}')

        while True:
            success, img = self.cap.read()
            if not success:
                if self.is_cam:
                    logging.info('Load image from source fail, wait 1 sec and retry.')
                    sleep(1)
                    continue
                else:
                    logging.info('Video is end.')
                    break

            small_img = self._resize_if_need(img)

            results = self.detector.detect(cv2.cvtColor(small_img, cv2.COLOR_BGR2RGB))

            if self.draw_locations:
                draw_locations(img, results, scale=int(1 / self.resize_factor) if self.resize_factor else 1)

            if self.show_window:
                show_to_window(img)

            if results and callable(self.event_func):
                self.event_func(results, img)

        self.cap.release()
        if self.show_window:
            cv2.destroyAllWindows()

    def _resize_if_need(self, img):
        if self.resize_factor:
            height, width, _ = img.shape
            return cv2.resize(img, (int(width*self.resize_factor), int(height*self.resize_factor)), interpolation=cv2.INTER_NEAREST)
