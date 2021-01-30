#!/usr/bin/env python3

import cv2
import rospy
import tf
import math
import numpy as np

from threading import Thread
from concurrent.futures import ThreadPoolExecutor
from turbojpeg import TurboJPEG, TJPF_GRAY
from image_geometry import PinholeCameraModel
from aruco_lib_adapter import Detector

from dt_class_utils import DTReminder
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType

from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import Transform, Vector3, Quaternion


class AprilTagDetector(DTROS):

    def __init__(self):
        super(AprilTagDetector, self).__init__(
            node_name='apriltag_detector_node',
            node_type=NodeType.PERCEPTION
        )
        # get static parameters
        self.ndetectors = rospy.get_param('~ndetectors', 4)
        self.tag_size = rospy.get_param('~tag_size', 0.065)
        self.rectify_alpha = rospy.get_param('~rectify_alpha', 0.0)
        self.detector_type = rospy.get_param('~detector_type', 'DFC')
        self.detector_config = rospy.get_param('~detector_config', 'config.yml')
        # dynamic parameter
        self.detection_freq = DTParam(
            '~detection_freq',
            default=-1,
            param_type=ParamType.INT,
            min_value=-1,
            max_value=30
        )
        self._detection_reminder = DTReminder(frequency=self.detection_freq.value)
        # camera info
        self._camera_parameters = None
        self._mapx, self._mapy = None, None
        # create detector object
        self._detectors = [Detector(
            detector_type=self.detector_type,
            marker_size=self.tag_size,
            config_file=self.detector_config
        ) for _ in range(self.ndetectors)]
        self._renderer_busy = False
        # create a CV bridge object
        self._jpeg = TurboJPEG()
        # create subscribers
        self._img_sub = rospy.Subscriber(
            '~image',
            CompressedImage,
            self._img_cb,
            queue_size=1,
            buff_size='20MB'
        )
        self._cinfo_sub = rospy.Subscriber(
            '~camera_info',
            CameraInfo,
            self._cinfo_cb,
            queue_size=1
        )
        # create publisher
        self._tag_pub = rospy.Publisher(
            '~detections',
            AprilTagDetectionArray,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION,
            dt_help='Tag detections',
        )
        self._img_pub = rospy.Publisher(
            '~detections/image/compressed',
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help='Camera image with tag publishs superimposed',
        )
        # create thread pool
        self._workers = ThreadPoolExecutor(self.ndetectors)
        self._tasks = [None] * self.ndetectors
        # create TF broadcaster
        self._tf_bcaster = tf.TransformBroadcaster()

    def on_shutdown(self):
        self.loginfo('Shutting down workers pool')
        self._workers.shutdown()

    def _cinfo_cb(self, msg):
        # create mapx and mapy
        H, W = msg.height, msg.width
        # create new camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        # find optimal rectified pinhole camera
        with self.profiler('/cb/camera_info/get_optimal_new_camera_matrix'):
            rect_K, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_model.K,
                self.camera_model.D,
                (W, H),
                self.rectify_alpha
            )
            # store new camera parameters
            self._camera_parameters = (rect_K[0, 0], rect_K[1, 1], rect_K[0, 2], rect_K[1, 2])
        # create rectification map
        with self.profiler('/cb/camera_info/init_undistort_rectify_map'):
            self._mapx, self._mapy = cv2.initUndistortRectifyMap(
                self.camera_model.K,
                self.camera_model.D,
                None,
                rect_K,
                (W, H),
                cv2.CV_32FC1
            )
        # once we got the camera info, we can stop the subscriber
        self.loginfo('Camera info message received. Unsubscribing from camera_info topic.')
        # noinspection PyBroadException
        try:
            self._cinfo_sub.shutdown()
        except BaseException:
            pass

    def _detect(self, detector_id, msg):
        # find out if visualization is needed
        rendering_needed = (self._img_pub.anybody_listening() and not self._renderer_busy)
        if rendering_needed:
            # turn image message into grayscale image
            with self.profiler('/cb/image/decode'):
                img = self._jpeg.decode(msg.data, pixel_format=TJPF_GRAY)
            # run input image through the rectification map
            with self.profiler('/cb/image/rectify'):
                img = cv2.remap(img, self._mapx, self._mapy, cv2.INTER_NEAREST)
            # prepare arguments
            rect_distortion = np.array([[0], [0], [0], [0]], dtype='float64')
            cp = self._camera_parameters
            rect_cam_matrix = np.array(
                [[cp[0], 0,     cp[2]],
                 [0,     cp[1], cp[3]],
                 [0,     0,     0]], dtype='float64')
            # detect tags
            with self.profiler('/cb/image/detection'):
                tags = self._detectors[detector_id].detect(
                    img, rect_cam_matrix, rect_distortion, img.shape[0], img.shape[1], uncompressed=True)
        else:
            # detect tags
            with self.profiler('/cb/image/detection'):
                tags = self._detectors[detector_id].detect(
                    msg, self.camera_model.K, self.camera_model.D, self.camera_model.height, self.camera_model.width)
        # pack detections into a message
        tags_msg = AprilTagDetectionArray()
        tags_msg.header.stamp = msg.header.stamp
        tags_msg.header.frame_id = msg.header.frame_id
        for tag in tags:
            # turn rotation matrix into quaternion
            q = _rvec2quat(tag.pose_R)
            p = tag.pose_t
            # create single tag detection object
            detection = AprilTagDetection(
                transform=Transform(
                    translation=Vector3(
                        x=p[0],
                        y=p[1],
                        z=p[2]
                    ),
                    rotation=Quaternion(
                        x=q[0],
                        y=q[1],
                        z=q[2],
                        w=q[3]
                    )
                ),
                tag_id=tag.tag_id,
                tag_family=str(tag.tag_family),
                hamming=tag.hamming,
                decision_margin=tag.decision_margin,
                homography=tag.homography.flatten().astype(np.float32).tolist(),
                center=tag.center.tolist(),
                corners=tag.corners.flatten().tolist(),
                pose_error=tag.pose_err
            )
            # add detection to array
            tags_msg.detections.append(detection)
            # publish tf
            self._tf_bcaster.sendTransform(
                p.tolist(),
                q.tolist(),
                msg.header.stamp,
                'tag/{:s}'.format(str(tag.tag_id)),
                msg.header.frame_id
            )
        # publish detections
        self._tag_pub.publish(tags_msg)
        # update healthy frequency metadata
        self._tag_pub.set_healthy_freq(self._img_sub.get_frequency())
        self._img_pub.set_healthy_freq(self._img_sub.get_frequency())
        # render visualization (if needed)
        if rendering_needed:
            self._renderer_busy = True
            Thread(target=self._render_detections, args=(msg, img, tags)).start()

    def _img_cb(self, msg):
        # make sure we have received camera info
        if self._camera_parameters is None:
            return
        # make sure we have a rectification map available
        if self._mapx is None or self._mapy is None:
            return
        # make sure somebody wants this
        if (not self._img_pub.anybody_listening()) and (not self._tag_pub.anybody_listening()):
            return
        # make sure this is a good time to detect (always keep this as last check)
        if not self._detection_reminder.is_time(frequency=self.detection_freq.value):
            return
        # make sure we are still running
        if self.is_shutdown:
            return
        # ---
        # find the first available worker (if any)
        for i in range(self.ndetectors):
            if self._tasks[i] is None or self._tasks[i].done():
                # submit this image to the pool
                self._tasks[i] = self._workers.submit(self._detect, i, msg)
                break

    def _render_detections(self, msg, img, detections):
        with self.profiler('/publishs_image'):
            # get a color buffer from the BW image
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            # draw each tag
            for detection in detections:
                for idx in range(len(detection.corners)):
                    cv2.line(
                        img,
                        tuple(detection.corners[idx - 1, :].astype(int)),
                        tuple(detection.corners[idx, :].astype(int)),
                        (0, 255, 0)
                    )
                # draw the tag ID
                cv2.putText(
                    img,
                    str(detection.tag_id),
                    org=(
                        detection.corners[0, 0].astype(int) + 10,
                        detection.corners[0, 1].astype(int) + 10
                    ),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8,
                    color=(0, 0, 255)
                )
            # pack image into a message
            img_msg = CompressedImage()
            img_msg.header.stamp = msg.header.stamp
            img_msg.header.frame_id = msg.header.frame_id
            img_msg.format = 'jpeg'
            img_msg.data = self._jpeg.encode(img)
        # ---
        self._img_pub.publish(img_msg)
        self._renderer_busy = False


def _rvec2quat(rvec):
    x = rvec[0]
    y = rvec[1]
    z = rvec[2]
    r = math.sqrt(x * x + y * y + z * z)
    if r < 0.00001:
        return np.array([1, 0, 0, 0])
    c = math.cos(r / 2)
    s = math.sin(r / 2)
    quat_x = c
    quat_y = s * z / r
    quat_z = -s * y / r
    quat_w = -s * x / r
    return np.array([quat_x, quat_y, quat_z, quat_w])


if __name__ == '__main__':
    node = AprilTagDetector()
    # spin forever
    rospy.spin()
