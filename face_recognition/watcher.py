import cv2
from time import sleep
from log import logging
from detector import FaceDetector, load_img_2_rgb
from utils import draw_locations, show_to_window

import threading
import queue

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
        
        return img

class MTWatcher(Watcher):
    img_queue_full_wait = 2  # sec

    def __init__(self, url, detector, thread_num=2, max_img_q=100, draw_locations=True, event_func=None, resize_factor=None):
        super().__init__(url, detector, draw_locations=draw_locations, event_func=event_func, resize_factor=resize_factor)
        self.thread_num = thread_num
        self.max_img_q = max_img_q

        self.queue = queue.Queue()
        self.is_file_end = False

        self.threads = []
        for i in range(0, self.thread_num):
            self.threads.append(threading.Thread(target=self._decode_job, args=(i,)))

        self.load_thread = threading.Thread(target=self._load_job)

    def run(self):
        logging.info(f'Start detect from file: {self.url}')

        # start load video job
        self.load_thread.start()

        # start decode jobs
        for t in self.threads:
            t.start()

        # wait jobs
        for t in self.threads + [self.load_thread]:
            t.join()

        self.cap.release()

        logging.info('All finish.')

    def _load_job(self):
        while True:
            if self.queue.qsize() < self.max_img_q:
                success, img = self.cap.read()
                if success:
                    self.queue.put(img)
                else:
                    logging.info('Video read is finish.')
                    self.is_file_end = True
                    break
            
            else:
                logging.debug('image queue is full, wait to load video.')
                sleep(self.img_queue_full_wait)

    def _decode_job(self, index):
        logging.info(f'start decode worker {index}.')
        
        while not self.is_file_end or self.queue.qsize() > 0:
            if self.queue.qsize() > 0:
                img = self.queue.get()
                self._decode(img)
        
        logging.info(f'decode worker {index} finish.')

    def _decode(self, img):
        small_img = self._resize_if_need(img)
        results = self.detector.detect(cv2.cvtColor(small_img, cv2.COLOR_BGR2RGB))

        if self.draw_locations:
            draw_locations(img, results, scale=1 / self.resize_factor if self.resize_factor is not None else 1)

        if results and callable(self.event_func):
            logging.info(results)
            self.event_func(results, img)