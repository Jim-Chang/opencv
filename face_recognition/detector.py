from typing import NamedTuple, List, Any
import cv2
import numpy as np
import face_recognition
import os
import time

IMAGE_FOLDER = 'images/'
RED_COLOR = (200, 58, 76)
WHITE_COLOR = (255, 255, 255)

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
        img = load_img_2_rgb(f'{IMAGE_FOLDER}{img_file}')
        encode = face_recognition.face_encodings(img)[0]

        faces.append(Face(
            name=img_file.split('.')[0],
            encode=encode,
        ))

    return faces

# img => RGB
def draw_location(img, match_result, scale=1):
    y1, x2, y2, x1 = match_result.location
    y1, x2, y2, x1 = y1 * scale, x2 * scale, y2 * scale, x1 * scale
    cv2.rectangle(img, (x1, y1), (x2, y2), RED_COLOR, 2)
    cv2.rectangle(img, (x1, y2 - 35), (x2, y2), RED_COLOR, cv2.FILLED)
    cv2.putText(img, match_result.name, (x1 + 10, y2-10), cv2.FONT_HERSHEY_COMPLEX, 0.8, WHITE_COLOR, 2)

class FaceDetector:

    def __init__(self, debug=False):
        self.debug = debug
        print('loading known face encodes...')

        _t = time.time()
        self.faces = load_faces()

        print(f'load finish!, use {time.time() - _t} seconds')

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
            matches = face_recognition.compare_faces(self.face_encodes, cur_face_encode)
            face_dis = face_recognition.face_distance(self.face_encodes, cur_face_encode)

            if self.debug:
                print('matches', list(zip(matches, self.face_names)))
                print('face_dis', list(zip(face_dis, self.face_names)))

            match_index = np.argmin(face_dis)
            if matches[match_index]:
                if self.debug:
                    print('match name:', self.faces[match_index].name)

                results.append(MatchResult(
                    name=self.faces[match_index].name,
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