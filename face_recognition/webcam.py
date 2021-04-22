import cv2
from detector import FaceDetector, load_img_2_rgb, draw_location

class Webcam:

    def __init__(self, url, detector, show_window=False, event_func=None, resize_factor=None):
        self.url = url
        self.detector = detector
        self.show_window = show_window
        self.event_func = event_func
        self.resize_factor = resize_factor

        self.cap = cv2.VideoCapture(url)

    def run(self):
        print(f'Start detect from webcam: {self.url}')
        while True:
            success, img = self.cap.read()
            ori_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            if self.resize_factor:
                height, width, _ = ori_img.shape
                img = cv2.resize(ori_img, (int(width*self.resize_factor), int(height*self.resize_factor)), interpolation=cv2.INTER_NEAREST)

            results = self.detector.detect(img)

            if callable(self.event_func):
                self.event_func(results)

            if self.show_window:
                for r in results:
                    draw_location(ori_img, r, scale=int(1 / self.resize_factor) if self.resize_factor else 1)

                cv2.imshow('cam', cv2.cvtColor(ori_img, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)


def main():
    cam = Webcam('http://192.168.68.58:8081/', FaceDetector(debug=True), True)
    cam.run()


if __name__ == '__main__':
    main()