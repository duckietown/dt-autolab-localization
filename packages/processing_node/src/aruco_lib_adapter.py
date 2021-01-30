#!/usr/bin/env python

"""Python wrapper for C++ library ArUco. This program creates two
classes that are used to detect apriltags and extract information from
them. Using this module, you can identify all apriltags visible in an
image, and get information about the location and orientation of the
tags.
"""

from __future__ import division
from __future__ import print_function

import os
import sys
import numpy

dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)  # add path to aruco_caller module
import aruco_caller


class Detection:
    """Data class for detected marker info"""

    def __init__(self):
        self.tag_family = None
        self.tag_id = None
        self.hamming = None
        self.decision_margin = None
        self.homography = None
        self.center = None
        self.corners = None
        self.pose_R = None
        self.pose_t = None
        self.pose_err = None

    def __str__(self):
        return (
            'Detection object:\n'
            'tag_family = {0}\ntag_id = {1}\nhamming = {2}\ndecision_margin = {3}\n'
            'homography = {4}\ncenter = {5}\ncorners = {6}\npose_R = {7}\npose_t = {8}\npose_err = {9}\n'.format(
                *map(str, [self.tag_family, self.tag_id, self.hamming, self.decision_margin,
                           self.homography, self.center, self.corners, self.pose_R, self.pose_t, self.pose_err])))

    def __repr__(self):
        return self.__str__()


class Detector(object):
    """Python wrapper for ArUco library (actually for aruco_caller.cpp)"""

    def __init__(self, detector_type='DFC', marker_size=0.065, config_file="config.yml"):
        """Initializer for ArUco library
        Args:
            detector_type (str): aruco_detector type; must be 'DFC' or 'STD'
            marker_size (float): marker side length
            config_file (str): path to configuration yml file
        """
        aruco_caller.aruco_init(detector_type, marker_size, config_file)

    def __del__(self):
        aruco_caller.aruco_destruct()

    def detect(self, image, camera_matrix, dist_coeffs, img_height, img_width, uncompressed=False):
        """Main function that calls marker detection from ArUco library
        Args:
            image (compressed_imgmsg or cv2_image): image for detection
            camera_matrix (3x3 numpy.array): camera matrix K ((fx, 0, cx), (0, fy, cy), (0, 0, 1))
            dist_coeffs (4x1 numpy.array): distortion coefficients D (k1, k2, p1, p2)
            img_height (float): image height
            img_width (float): image width
            uncompressed (bool): flag for already uncompressed images
        Returns:
            list: list of Detection objects containing info about each detected marker
        """

        calib_dict = {
            "height": img_height, "width": img_width,
            "camera_matrix00": camera_matrix.item(0, 0), "camera_matrix02": camera_matrix.item(0, 2),
            "camera_matrix11": camera_matrix.item(1, 1), "camera_matrix12": camera_matrix.item(1, 2),
            "distortion0": dist_coeffs.item(0, 0), "distortion1": dist_coeffs.item(1, 0),
            "distortion2": dist_coeffs.item(2, 0), "distortion3": dist_coeffs.item(3, 0)
        }

        # choose right function
        if uncompressed:
            aruco_detect_func = aruco_caller.aruco_detect_and_estimate
        else:
            aruco_detect_func = aruco_caller.aruco_imdecode_detect_and_estimate
        # detect markers in the image
        detections = aruco_detect_func(
            calib_dict, numpy.ndarray(shape=(1, len(image.data)), dtype=numpy.uint8, buffer=image.data))

        return_info = []

        for marker in detections:
            # sometimes marker is detected but its pose can't be estimated
            if len(marker["rvec"]) != 3:
                continue

            detection = Detection()
            detection.tag_family = marker["tag_family"]
            detection.tag_id = marker["tag_id"]
            detection.corners = numpy.array(marker["corners"], dtype='float32').reshape((4, 2))
            detection.pose_R = numpy.array(marker["rvec"], dtype='float64')
            detection.pose_t = numpy.array(marker["tvec"], dtype='float64')

            # these fields are unsupported now
            detection.hamming = -1
            detection.decision_margin = -1.0
            detection.homography = numpy.zeros((3, 3), dtype='float32')
            detection.center = numpy.zeros(2, dtype='float32')
            detection.pose_err = -1.0

            return_info.append(detection)

        return return_info


# AUTOTESTS ###################################################################

def imshow_with_tags(img, tags, window_name):
    """Helper function for tests that shows drawn tags on the provided image"""

    colored_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(colored_img,
                     tuple(tag.corners[idx - 1, :].astype(int)),
                     tuple(tag.corners[idx, :].astype(int)),
                     (0, 255, 0))

        cv2.putText(colored_img, str(tag.tag_id),
                    org=(tag.corners[0, 0].astype(int) + 10,
                         tag.corners[0, 1].astype(int) + 10),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255))

    cv2.imshow(window_name, colored_img)
    k = cv2.waitKey(0)
    if k == 27:  # wait for ESC key to exit
        cv2.destroyAllWindows()


if __name__ == '__main__':

    test_images_path = '../apriltags3-py/test'
    distCoeffs = [0, 0, 0, 0]

    visualization = True
    try:
        import cv2
    except:
        raise Exception('You need cv2 in order to run the demo. However, you can still use the library without it.')

    try:
        from cv2 import imshow
    except:
        print("The function imshow was not implemented in this installation. Rebuild OpenCV from source to use it")
        print("VIsualization will be disabled.")
        visualization = False

    try:
        import yaml
    except:
        raise Exception('You need yaml in order to run the tests. However, you can still use the library without it.')

    with open("test_config.yml", 'w') as f:
        f.write(
            """%YAML:1.0
            ---
            aruco-dictionary: "TAG36h11"
            aruco-maxThreads: 1
            dcf-detectRate: 1""")
    at_detector = Detector(config_file="test_config.yml")
    os.remove("test_config.yml")

    with open(test_images_path + '/test_info.yaml', 'r') as stream:
        parameters = yaml.load(stream)

    #### TEST WITH THE SAMPLE IMAGE ####

    print("\n\nTESTING WITH A SAMPLE IMAGE")

    img = cv2.imread(test_images_path+'/'+parameters['sample_test']['file'], cv2.IMREAD_GRAYSCALE)
    cameraMatrix = numpy.array(parameters['sample_test']['K']).reshape((3,3))

    tags = at_detector.detect(img, cameraMatrix, distCoeffs, img.shape[0], img.shape[1], True)
    print(tags)

    if visualization:
        cv2.imshow('Original image',img)
        imshow_with_tags(img, tags, 'Detected tags')

    #### TEST WITH THE ROTATION IMAGES ####

    import time

    print("\n\nTESTING WITH ROTATION IMAGES")

    time_num = 0
    time_sum = 0

    test_images_path = '../apriltags3-py/test'
    image_names = parameters['rotation_test']['files']

    for image_name in image_names:
        print("Testing image ", image_name)
        ab_path = test_images_path + '/' + image_name
        if(not os.path.isfile(ab_path)):
            continue
        groundtruth = float(image_name.split('_')[-1].split('.')[0])  # name of test image should be set to its groundtruth

        parameters['rotation_test']['rotz'] = groundtruth
        cameraMatrix = numpy.array(parameters['rotation_test']['K']).reshape((3,3))

        img = cv2.imread(ab_path, cv2.IMREAD_GRAYSCALE)

        start = time.time()
        tags = at_detector.detect(img, cameraMatrix, distCoeffs, img.shape[0], img.shape[1], True)
        time_sum+=time.time()-start
        time_num+=1

        if tags:
            print(tags[0].pose_t, parameters['rotation_test']['posx'], parameters['rotation_test']['posy'], parameters['rotation_test']['posz'])
            print(tags[0].pose_R, parameters['rotation_test']['rotx'], parameters['rotation_test']['roty'], parameters['rotation_test']['rotz'])
        else:
            print("Tags not found")

        if visualization:
            imshow_with_tags(img, tags, "Detected tags for " + image_name)

    print("AVG time per detection: ", time_sum/time_num)

    #### TEST WITH MULTIPLE TAGS IMAGES ####

    print("\n\nTESTING WITH MULTIPLE TAGS IMAGES")

    time_num = 0
    time_sum = 0

    image_names = parameters['multiple_tags_test']['files']

    for image_name in image_names:
        print("Testing image ", image_name)
        ab_path = test_images_path + '/' + image_name
        if(not os.path.isfile(ab_path)):
            continue

        cameraMatrix = numpy.array(parameters['multiple_tags_test']['K']).reshape((3,3))

        img = cv2.imread(ab_path, cv2.IMREAD_GRAYSCALE)

        start = time.time()
        tags = at_detector.detect(img, cameraMatrix, distCoeffs, img.shape[0], img.shape[1], True)
        time_sum+=time.time()-start
        time_num+=1

        tag_ids = [tag.tag_id for tag in tags]
        print(len(tags), " tags found: ", tag_ids)

        if visualization:
            imshow_with_tags(img, tags, "Detected tags for " + image_name)

    print("AVG time per detection: ", time_sum/time_num)
