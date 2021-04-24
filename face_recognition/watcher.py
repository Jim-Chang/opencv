import cv2
from time import sleep
from log import logging
from detector import FaceDetector, load_img_2_rgb
from displayer import draw_locations, show_to_window

class Watcher:

    def __init__(self, url, detector, show_window=False, event_func=None, resize_factor=None):
        self.url = url
        self.detector = detector
        self.show_window = show_window
        self.event_func = event_func
        self.resize_factor = resize_factor

        self.cap = cv2.VideoCapture(url)

    def run(self):
        logging.info(f'Start detect from webcam: {self.url}')

        while True:
            img = self._read_img()
            small_img = self._resize_if_need(img)

            results = self.detector.detect(small_img)

            if results and callable(self.event_func):
                self.event_func(results)

            if self.show_window:
                draw_locations(img, results, scale=int(1 / self.resize_factor) if self.resize_factor else 1)
                show_to_window(img)

    def _read_img(self):
        success = False
        while success is False:
            success, img = self.cap.read()
            if success:
                return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                logging.info('Load image from source fail, wait 1 sec and retry.')
                sleep(1000)

    def _resize_if_need(self, img):
        if self.resize_factor:
            height, width, _ = img.shape
            return cv2.resize(img, (int(width*self.resize_factor), int(height*self.resize_factor)), interpolation=cv2.INTER_NEAREST)


def main():
    cam = Webcam('http://192.168.68.58:8081/', FaceDetector(debug=True), True)
    cam.run()


if __name__ == '__main__':
    main()