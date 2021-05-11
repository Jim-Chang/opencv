from typing import NamedTuple, List, Any
import cv2
import numpy as np
import face_recognition
import os
import time
from log import logging
from utils import IMAGE_FOLDER, FACE_DAT_FILE
import pickle
try:
    import mediapipe as mp
except:
    pass

class Face(NamedTuple):
    name: str
    encode: Any
    source: str  # jpg file name

class MatchResult(NamedTuple):
    name: str
    location: Any
    is_unknown: bool


def load_img_2_rgb(path):
    img = cv2.imread(path)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def build_faces_dat():
    file_list = _ls_face_image_names()
    dat_path = f'{IMAGE_FOLDER}/{FACE_DAT_FILE}'

    try:
        faces = load_faces_from_dat()
    except Exception as e:
        logging.warning('dat file not found')
        faces = []

    # 刪除 img 檔案不存在的 face
    for face in faces:
        if face.source not in file_list:
            faces.remove(face)
            logging.info(f'Delete encoding data of ${face.source} from dat')

    logging.info('Load face img and encode if need')
    faces += load_faces_from_img([f.source for f in faces])

    logging.info('Save face encodings to dat file')
    with open(f'{IMAGE_FOLDER}/{FACE_DAT_FILE}', 'wb') as f:
        pickle.dump(faces, f)

    logging.info('Finish')


def _ls_face_image_names() -> List[str]:
    return filter(lambda f: '.jpg' in f, os.listdir(IMAGE_FOLDER))


def load_faces_from_dat() -> List[Face]:
    faces = []
    dat_path = f'{IMAGE_FOLDER}/{FACE_DAT_FILE}'

    if os.path.exists(dat_path):
        logging.info('Load face encodings from dat file')
        with open(dat_path, 'rb') as f:
            faces = pickle.load(f)

        return faces

    raise Exception('dat file not exist')


def load_faces_from_img(exclude_filenames=[]) -> List[Face]:
    faces = []
    file_list = _ls_face_image_names()

    for img_file in file_list:
        # 只讀取 jpg，且在 dat 檔裡面沒有的
        if img_file not in exclude_filenames:
            img = load_img_2_rgb(f'{IMAGE_FOLDER}/{img_file}')
            encode = face_recognition.face_encodings(img)[0]

            faces.append(Face(
                name=img_file.split('.')[0].split('_')[0],   # Jim_1.jpg, Jim_2.jpg...
                encode=encode,
                source=img_file,
            ))

    return faces


class FaceDetector:

    def __init__(self, tolerance=0.6, debug=False, detect_by='face_recognition'):
        self.tolerance = tolerance
        self.debug = debug
        self.detect_by = detect_by
        logging.info('loading known face encodes...')

        if detect_by == 'mediapipe':
            self.face_detection = mp.solutions.face_detection.FaceDetection(min_detection_confidence=0.5)

        _t = time.time()

        try:
            self.faces = load_faces_from_dat()
        except Exception as e:
            logging.warning('Load face from dat fail, try to load from img')
            self.faces = load_faces_from_img()

        logging.info(f'load finish!, use {time.time() - _t} seconds')

    @property
    def face_encodes(self):
        if not hasattr(self, '_face_encodes'):
            self._face_encodes = [f.encode for f in self.faces]

        return self._face_encodes

    @property
    def face_names(self):
        if not hasattr(self, '_face_names'):
            self._face_names = [f.name for f in self.faces]

        return self._face_names

    def _face_locations(self, img):
        if self.detect_by == 'face_recognition':
            return face_recognition.face_locations(img)

        elif self.detect_by == 'mediapipe':
            results = self.face_detection.process(img)

            if not results.detections:
                return []

            locs = []
            height, width, _ = img.shape

            for detection in results.detections:
                box = detection.location_data.relative_bounding_box
                x1, x2, y1, y2 = int(width * box.xmin), int(width * (box.xmin + box.width)), int(height * box.ymin), int(height * (box.ymin + box.height))
                locs.append((y1, x2, y2, x1))

            return locs

        else:
            raise Exception('detect method not support')

    # img: RGB
    def detect(self, img) -> List[MatchResult]:
        results = []

        cur_face_locs = self._face_locations(img)
        if not cur_face_locs:
            return []

        cur_face_encodes = face_recognition.face_encodings(img, cur_face_locs)

        for cur_face_loc, cur_face_encode in zip(cur_face_locs, cur_face_encodes):
            face_dis = face_recognition.face_distance(self.face_encodes, cur_face_encode)

            if self.debug:
                print('face_dis', list(zip(face_dis, self.face_names)))

            min_dis_index = np.argmin(face_dis)
            if face_dis[min_dis_index] <= self.tolerance:
                results.append(MatchResult(
                    name=self.faces[min_dis_index].name,
                    location=cur_face_loc,
                    is_unknown=False
                ))

            else:
                results.append(MatchResult(
                    name='unknown',
                    location=cur_face_loc,
                    is_unknown=True
                ))

        return results

    def close(self):
        if self.detect_by == 'mediapipe':
            self.face_detection.close()