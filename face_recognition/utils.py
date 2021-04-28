import cv2
import numpy as np
import json

IMAGE_FOLDER = 'images'

# BGR
RED_COLOR = (76, 58, 200)
WHITE_COLOR = (255, 255, 255)


# nparray
def im_nparr_2_bytes(img):
    return cv2.imencode('.jpg', img)[1].tobytes()


# data: bytes (jpg)
def im_bytes_2_nparr(data):
    nparr = np.frombuffer(data, np.uint8)
    return cv2.imdecode(nparr, cv2.IMREAD_COLOR)


# imgs: nparray list
def im_concat(imgs):
    return np.concatenate(imgs, axis=1)


# img => BGR, nparray
def draw_locations(img, match_results, scale=1):
    for match_result in match_results:
        y1, x2, y2, x1 = match_result.location
        y1, x2, y2, x1 = round(y1 * scale), round(x2 * scale), round(y2 * scale), round(x1 * scale)
        cv2.rectangle(img, (x1, y1), (x2, y2), RED_COLOR, 2)
        cv2.rectangle(img, (x1, y2 + 35), (x2, y2), RED_COLOR, cv2.FILLED)
        cv2.putText(img, match_result.name, (x1 + 10, y2 + 25), cv2.FONT_HERSHEY_COMPLEX, 0.8, WHITE_COLOR, 2)


# img => RGB, nparray
def show_to_window(img, window='cam', wait=1):
    cv2.imshow(window, img)
    cv2.waitKey(wait)


class MetaData:
    def __init__(self, data):
        self.data = data

    def get_nickname(self, name) -> str:
        return self.data['nickname_map'].get(name, '')

    def get_authorize_name_set(self):
        return set(self.data['authorize_list'])
    

def load_meta_data() -> MetaData:
    with open(f'{IMAGE_FOLDER}/meta_data.json', 'r') as f:
        data = f.read()
        return MetaData(json.loads(data))
