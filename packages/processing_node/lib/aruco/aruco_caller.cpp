/** ArUco library adapter for python
 */

#include <iostream>
#include <opencv2/core.hpp>
#include <boost/python.hpp>
#include <boost/numpy/ndarray.hpp>
#include <boost/numpy.hpp>
#include "aruco.h"
#include "dcf/dcfmarkertracker.h"

using namespace std;
using namespace boost::python;


class IArucoDetector {
public:
    virtual vector<Marker> set_params_detect_and_estimate(const cv::Mat &image, const CameraParameters &cam_params) = 0;

    virtual ~IArucoDetector() = default;
};

class DFCArucoDetector : public IArucoDetector {
public:
    DFCArucoDetector(float marker_size, const string &config_file_string) :
            marker_size(marker_size) {
        marker_tracker.loadParamsFromFile(config_file_string);
    }

    vector<Marker> set_params_detect_and_estimate(const cv::Mat &image, const CameraParameters &cam_params) override {
        marker_tracker.setParams(cam_params, marker_size);
        map<int, cv::Ptr<TrackerImpl>> set_trackers = marker_tracker.track(image, 0.1);
        marker_tracker.estimatePose();
        vector<Marker> markers_vec(set_trackers.size());
        int i = 0;
        for (const auto &t : set_trackers) {
            markers_vec[i] = t.second->getMarker();
            i++;
        }
        return markers_vec;
    }

    ~DFCArucoDetector() override = default;

private:
    DFCMarkerTracker marker_tracker;
    float marker_size;
};

class STDArucoDetector : public IArucoDetector {
public:
    STDArucoDetector(float marker_size, const string &config_file_string) :
            marker_size(marker_size) {
        marker_detector.loadParamsFromFile(config_file_string);

        MarkerDetector::Params params = marker_detector.getParameters();
        marker_detector.setDetectionMode(params.detectMode, params.minSize);
        params.setCornerRefinementMethod(params.cornerRefinementM);
    }

    vector<Marker> set_params_detect_and_estimate(const cv::Mat &image, const CameraParameters &cam_params) override {
        return marker_detector.detect(image, cam_params, marker_size);
    }

    ~STDArucoDetector() override = default;

private:
    MarkerDetector marker_detector;
    float marker_size;
};


IArucoDetector *aruco_detector = nullptr;

/**Initialize parameters for ArUco library and create appropriate aruco_detector object
 * @param detector_type python str with aruco_detector type; must be 'DFC' or 'STD'
 * @param marker_size marker side length
 * @param config_file_str python str with path to configuration yml file
 */
void aruco_init(const str &detector_type, float marker_size, const str &config_file_str) {
    string detector_type_string = extract<string>(detector_type);
    string config_file_string = extract<string>(config_file_str);
    if (detector_type_string == "DFC") {
        aruco_detector = new DFCArucoDetector(marker_size, config_file_string);
    } else if (detector_type_string == "STD") {
        aruco_detector = new STDArucoDetector(marker_size, config_file_string);
    } else {
        printf("Unknown detector type: %s, use 'DFC' or 'STD'\n", detector_type_string.c_str());
    }
}

void aruco_destruct() {
    delete aruco_detector;
}

/**Helper function for aruco_imdecode_detect_and_estimate and aruco_detect_and_estimate
 */
list aruco_detect_and_estimate_helper(const dict &calib_dict, const cv::Mat &image) {
    if (aruco_detector == nullptr) {
        printf("Must call aruco_init first\n");
        return list();
    }

    cv::Mat camera_matrix(3, 3, CV_32F);
    camera_matrix.at<float>(0, 0) = extract<float>(calib_dict["camera_matrix00"]);
    camera_matrix.at<float>(0, 2) = extract<float>(calib_dict["camera_matrix02"]);
    camera_matrix.at<float>(1, 1) = extract<float>(calib_dict["camera_matrix11"]);
    camera_matrix.at<float>(1, 2) = extract<float>(calib_dict["camera_matrix12"]);

    cv::Mat distortion(4, 1, CV_32F);
    distortion.at<float>(0, 0) = extract<float>(calib_dict["distortion0"]);
    distortion.at<float>(1, 0) = extract<float>(calib_dict["distortion1"]);
    distortion.at<float>(2, 0) = extract<float>(calib_dict["distortion2"]);
    distortion.at<float>(3, 0) = extract<float>(calib_dict["distortion3"]);

    cv::Size image_size;
    image_size.height = extract<int>(calib_dict["height"]);
    image_size.width = extract<int>(calib_dict["width"]);

    CameraParameters cam_params;
    try {
        cam_params = CameraParameters(camera_matrix, distortion, image_size);
    } catch (cv::Exception &e) {
        printf("\nFailed: [%s]\n", e.msg.c_str());
    } catch (std::exception &e) {
        printf("\nFailed: [%s]\n", e.what());
    }

    vector<Marker> markers_vector = aruco_detector->set_params_detect_and_estimate(image, cam_params);

    list markers_list;

    for (const Marker &marker : markers_vector) {
        dict marker_dict;

        marker_dict["tag_id"] = marker.id;
        marker_dict["tag_family"] = marker.dict_info;


        list marker_corners;
        for (int i = 0; i < 4; i++) {
            marker_corners.append(marker[i].x);
            marker_corners.append(marker[i].y);
        }

        marker_dict["corners"] = marker_corners;


        list marker_rvec;
        list marker_tvec;
        if (!marker.Rvec.empty() && !marker.Tvec.empty()) {
            for (int i = 0; i < 3; i++) {
                marker_rvec.append(marker.Rvec.at<float>(i));
                marker_tvec.append(marker.Tvec.at<float>(i));
            }
        }
        marker_dict["rvec"] = marker_rvec;
        marker_dict["tvec"] = marker_tvec;


        markers_list.append(marker_dict);
    }

    return markers_list;
}

/**Exactly the same with aruco_detect_and_estimate, but img_data contains compressed image
 */
list aruco_imdecode_detect_and_estimate(const dict &calib_dict, const boost::numpy::ndarray &img_data) {
    cv::Mat buf(1, img_data.get_shape()[1], CV_8U, img_data.get_data());
    cv::Mat image = cv::imdecode(buf, 4);

    return aruco_detect_and_estimate_helper(calib_dict, image);
}

/**Detects markers in the image and estimates their poses
 * @param calib_dict python dict with intrinsic calibration data
 * @param img_data numpy array of image data; example of param construction:
 *        numpy.ndarray(shape=(1, len(image.data)), dtype=numpy.uint8, buffer=image.data)
 * @return python list containing info about each detected marker
 */
list aruco_detect_and_estimate(const dict &calib_dict, const boost::numpy::ndarray &img_data) {
    cv::Size image_size;
    image_size.height = extract<int>(calib_dict["height"]);
    image_size.width = extract<int>(calib_dict["width"]);
    cv::Mat image(image_size.height, image_size.width, CV_8U, img_data.get_data());

    return aruco_detect_and_estimate_helper(calib_dict, image);
}

/**Declares functions visible in python
 */
BOOST_PYTHON_MODULE (aruco_caller) {
    boost::numpy::initialize();
    def("aruco_init", aruco_init);
    def("aruco_destruct", aruco_destruct);
    def("aruco_detect_and_estimate", aruco_detect_and_estimate);
    def("aruco_imdecode_detect_and_estimate", aruco_imdecode_detect_and_estimate);
}