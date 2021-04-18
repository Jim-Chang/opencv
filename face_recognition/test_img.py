import sys
import cv2

from detector import FaceDetector, load_img_2_rgb, draw_location

TEST_IMG_FOLDER = 'test_images/'

def main(file_name):
    test_img = load_img_2_rgb(TEST_IMG_FOLDER + file_name)
    detector = FaceDetector(debug=True)
    results = detector.detect(test_img)
    for r in results:
        draw_location(test_img, r)

    cv2.imshow('result', cv2.cvtColor(test_img, cv2.COLOR_RGB2BGR))
    cv2.waitKey()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv[1])