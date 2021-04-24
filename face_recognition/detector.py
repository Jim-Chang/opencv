from typing import NamedTuple, List, Any
import cv2
import numpy as np
import face_recognition
import os
import time
from log import logging

IMAGE_FOLDER = 'images'

class Face(NamedTuple):
    name: str
    encode: Any

class MatchResult(NamedTuple):
    name: str
    location: Any
    unknown: bool


def load_img_2_rgb(path):
    img = cv2.imread(path)
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def load_faces() -> List[Face]:
    faces = []
    file_list = os.listdir(IMAGE_FOLDER)

    for img_file in file_list:
        img = load_img_2_rgb(f'{IMAGE_FOLDER}/{img_file}')
        encode = face_recognition.face_encodings(img)[0]

        faces.append(Face(
            name=img_file.split('.')[0],
            encode=encode,
        ))

    return faces

class FaceDetector:

    def __init__(self, tolerance=0.6, debug=False):
        self.tolerance = tolerance
        self.debug = debug
        logging.info('loading known face encodes...')

        _t = time.time()
        self.faces = load_faces()

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

    def detect(self, img) -> List[MatchResult]:
        results = []

        cur_face_locs = face_recognition.face_locations(img)
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
                    unknown=False
                ))

            else:
                results.append(MatchResult(
                    name='unknown',
                    location=cur_face_loc,
                    unknown=True
                ))

        return results