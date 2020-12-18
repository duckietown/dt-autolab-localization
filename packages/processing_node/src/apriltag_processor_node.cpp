#include <ros/ros.h>

#include <chrono>
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <boost/lexical_cast.hpp>
#include <opencv2/core.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>

#include <duckietown_msgs/AprilTagDetection.h>
#include <duckietown_msgs/AprilTagDetectionArray.h>
#include <dcf/dcfmarkertracker.h>


using namespace std;

// class that counts the average frequency of the tick() function call
class timer_avg {
public:
    explicit timer_avg(int n = 100) :
            n(n), k(0) {
        for (int i = 0; i < n; i++) {
            times.push(chrono::high_resolution_clock::now());
        }
    }

    inline void tick() {
        k++;
        times.push(chrono::high_resolution_clock::now());
        times.pop();
    }

    double avg() {
        return chrono::duration_cast<chrono::microseconds>(times.back() - times.front()).count()
               * 1e-6 / max(1, min(n, k));
    }

private:
    queue<chrono::high_resolution_clock::time_point> times;
    int n, k;
};

// concurrent queue with blocking on the pop() function call
template<typename T>
class concurrent_blocking_queue {
public:
    concurrent_blocking_queue() {
        closed = false;
    }

    bool pop(T &item) {
        unique_lock<mutex> lock(m);
        while (q.empty() && !closed) {
            c.wait(lock);
        }
        if (closed) {
            return false;
        }
        item = q.front();
        q.pop();
        return true;
    }

    void push(const T &item) {
        unique_lock<mutex> lock(m);
        q.push(item);
        lock.unlock();
        c.notify_one();
    }

    void push(T &&item) {
        unique_lock<mutex> lock(m);
        q.push(move(item));
        lock.unlock();
        c.notify_one();
    }

    void close() {
        closed = true;
        c.notify_all();
    }

    int size() {
        unique_lock<mutex> lock(m);
        return q.size();
    }

private:
    queue<T> q;
    mutex m;
    condition_variable c;
    bool closed;
};

// dataclass for storing detection info about one tag
struct tag_data {
    int tag_id = 0;
    string tag_family;
    vector<cv::Point2f> corners;
    cv::Mat tvec, rvec;
};

// dataclass for storing detection info about all tags
struct tags_data {
    vector<tag_data> tags;
    std_msgs::Header header;
};

// dataclass for marker tracker params
struct marker_tracker_params {
    int height = 1296, width = 976;
    string config_file;
    float marker_size = 0.065;
    vector<double> K, D;
};

// construct CameraParameters object from marker_tracker_params object
CameraParameters load_params(const marker_tracker_params &params) {
    cv::Size image_size;
    image_size.height = params.height;
    image_size.width = params.width;

    int cols = 3;
    int rows = 3;
    cv::Mat camera_matrix(rows, cols, CV_32F);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            camera_matrix.at<float>(i, j) = (float) params.K[i * cols + j];
        }
    }

    cols = params.D.size();
    cv::Mat distortion(cols, 1, CV_32F);
    for (int j = 0; j < cols; j++) {
        distortion.at<float>(j, 0) = (float) params.D[j];
    }

    return CameraParameters(camera_matrix, distortion, image_size);
}

// gather params into marker_tracker_params object
marker_tracker_params gather_params(const sensor_msgs::CameraInfo &cam_info,
                                    const string &config_file, float marker_size) {
    marker_tracker_params params;
    params.height = cam_info.height;
    params.width = cam_info.width;
    params.config_file = config_file;
    params.marker_size = marker_size;

    std::vector<double> K(9);
    for (int i = 0; i < 9; i++) {
        K[i] = cam_info.K[i];
    }
    std::vector<double> D = cam_info.D;

    params.K = K;
    params.D = D;

    return params;
}

// convert cv::Mat rvec into geometry_msgs::Quaternion
geometry_msgs::Quaternion rvec2quat(cv::Mat rvec) {
    geometry_msgs::Quaternion quat;
    double x = rvec.at<float>(0);
    double y = rvec.at<float>(1);
    double z = rvec.at<float>(2);
    double r = sqrt(x * x + y * y + z * z);
    if (r < 0.00001) {
        quat.x = 1;
        return quat;
    }
    double c = cos(r / 2);
    double s = sin(r / 2);
    quat.x = c;
    quat.y = s * z / r;
    quat.z = -s * y / r;
    quat.w = -s * x / r;
    return quat;
}

// get string environment variable
string getenv(const char *variable_name, const char *default_value) {
    const char *value = getenv(variable_name);
    return value ? value : default_value;
}

// main processing class
class apriltag_processor {
public:
    explicit apriltag_processor(ros::NodeHandle &node_handle) :
            processors_num(4) {

        device_name = getenv("ACQ_DEVICE_NAME", "watchtower10");
        logging = getenv("ACQ_LOGGING", "false") == "true";

        string config_file, camera_image_topic, camera_info_topic, apriltags_topic;
        float marker_size;

        node_handle.param("config_file", config_file, string("config.yml"));
        node_handle.param("camera_image_topic", camera_image_topic, string("camera_node/image/compressed"));
        node_handle.param("camera_info_topic", camera_info_topic, string("camera_node/camera_info"));
        node_handle.param("apriltags_topic", apriltags_topic, string("config.yml"));
        node_handle.param("processors_num", processors_num, 4);
        node_handle.param("marker_size", marker_size, 0.065f);

        string img_comp_topic = "/" + device_name + "/" + camera_image_topic;
        string cam_info_topic = "/" + device_name + "/" + camera_info_topic;

        comp_img_sub = node_handle.subscribe(img_comp_topic, 200,
                                             &apriltag_processor::img_comp_callback, this);
        cam_info_sub = node_handle.subscribe(cam_info_topic, 1,
                                             &apriltag_processor::cam_info_callback, this);

        apriltags_pub = node_handle.advertise<duckietown_msgs::AprilTagDetectionArray>(apriltags_topic, 20);

        cout.precision(5);
        cout << fixed;

        ros::Rate rate(10);
        while (!was_cam_info && ros::ok()) {
            if (logging) {
                cout << "wait camera_info" << endl;
            }
            ros::spinOnce();
            rate.sleep();
        }

        if (!ros::ok()) {
            return;
        }

        marker_tracker_params params = gather_params(cam_info, config_file, marker_size);

        for (int i = 0; i < processors_num; i++) {
            img_processors.emplace_back(thread(&apriltag_processor::img_processor, this, params));
        }
        pub_thread = thread(&apriltag_processor::publisher, this);
    }

    // main image processing function
    void img_processor(const marker_tracker_params &params) {
        DFCMarkerTracker marker_tracker;
        CameraParameters cam_params = load_params(params);
        marker_tracker.setParams(cam_params, params.marker_size);
        marker_tracker.loadParamsFromFile(params.config_file);
        pair<std_msgs::Header, cv::Mat> header_image;
        while (img_queue.pop(header_image)) {
            map<int, cv::Ptr<TrackerImpl>> set_trackers = marker_tracker.track(header_image.second, 0.1);
            marker_tracker.estimatePose();
            vector<Marker> markers_vec(set_trackers.size());
            tags_data tags;
            for (const auto &t : set_trackers) {
                Marker &marker = t.second->getMarker();

                tag_data tag;
                tag.tag_id = marker.id;
                tag.tag_family = marker.dict_info;
                tag.tvec = marker.Tvec;
                tag.rvec = marker.Rvec;
                for (int j = 0; j < marker.size(); j++) {
                    tag.corners.push_back(marker[j]);
                }
                tags.tags.push_back(tag);
            }
            if (!tags.tags.empty()) {
                tags.header = header_image.first;
                tags_queue.push(tags);
            }
        }
    }

    // tags publisher
    void publisher() {
        cv::Mat image;
        tags_data tags;
        geometry_msgs::Vector3 vec3;
        geometry_msgs::Quaternion quat;
        int tag_msg_seq = 0;
        while (tags_queue.pop(tags)) {
            duckietown_msgs::AprilTagDetectionArray apriltags_msg;

            ros::Time t = ros::Time::now();
            for (auto tag : tags.tags) {
                if (tag.tvec.cols * tag.tvec.rows != 3 || tag.rvec.cols * tag.rvec.rows != 3) {
                    if (logging) {
                        cout << "estimation_error: tvec=" << tag.tvec << "  rvec=" << tag.rvec << endl;
                    }
                    continue;
                }
                duckietown_msgs::AprilTagDetection apriltag_msg;
                apriltag_msg.tag_id = tag.tag_id;
                apriltag_msg.tag_family = tag.tag_family;
                for (int i = 0; i < 4; i++) {
                    apriltag_msg.corners[2 * i + 0] = tag.corners[i].x;
                    apriltag_msg.corners[2 * i + 1] = tag.corners[i].y;
                }
                vec3.x = tag.tvec.at<float>(0);
                vec3.y = tag.tvec.at<float>(1);
                vec3.z = tag.tvec.at<float>(2);
                apriltag_msg.transform.translation = vec3;
                apriltag_msg.transform.rotation = rvec2quat(tag.rvec);

                apriltags_msg.detections.push_back(apriltag_msg);
            }

            apriltags_msg.header.stamp = t;
            apriltags_msg.header.seq = tag_msg_seq++;
            apriltags_msg.header.frame_id = device_name;
            apriltags_pub.publish(apriltags_msg);
        }
    }

    void img_comp_callback(const sensor_msgs::CompressedImageConstPtr &img) {
        if (!was_cam_info) {
            return;
        }
        static int iter = 0;
        static timer_avg timer;

        int rows = 1;
        int cols = (int) (cam_info.height * cam_info.width);
        cv::Mat buf(rows, cols, CV_8U, const_cast<uint8_t *>(img->data.data()));
        cv::Mat color_image = cv::imdecode(buf, cv::IMREAD_COLOR);
        cv::Mat image;
        cv::cvtColor(color_image, image, cv::COLOR_BGR2GRAY);

        std_msgs::Header header(img->header);
        img_queue.push(make_pair(header, move(image)));

        timer.tick();
        if (logging) {
            cout << setw(6) << ++iter << setw(12) << img_queue.size()
                 << fixed << setw(15) << setprecision(2) << timer.avg() * 1000 << endl;
        }
    }

    void cam_info_callback(const sensor_msgs::CameraInfo &cam_info_) {
        if (!was_cam_info && logging) {
            cout << setw(6) << "iters" << setw(12) << "img_queue" << setw(15) << "comp_cb_freq" << endl;
        }
        cam_info = cam_info_;
        was_cam_info = true;
    }

    ~apriltag_processor() {
        tags_queue.close();
        img_queue.close();
        for (int i = 0; i < processors_num; i++) {
            img_processors[i].join();
        }
        pub_thread.join();
    }

private:
    concurrent_blocking_queue<pair<std_msgs::Header, cv::Mat>> img_queue;
    concurrent_blocking_queue<tags_data> tags_queue;

    int processors_num;
    vector<thread> img_processors;
    thread pub_thread;

    sensor_msgs::CameraInfo cam_info;
    bool was_cam_info = false;

    string device_name;
    ros::Subscriber comp_img_sub, cam_info_sub;
    ros::Publisher apriltags_pub;

    bool logging;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "apriltag_processor_node");
    ros::NodeHandle node_handle("~");

    apriltag_processor ap(node_handle);
    ros::spin();
}