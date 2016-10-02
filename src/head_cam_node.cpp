#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <math.h>


namespace HeadCam {

class HeadCamNode {
public :

    enum {
        NONE = 0,
        KWP = 1,
        FISHEYE = 2
    };


    ros::NodeHandle node_;
    image_transport::Publisher  publisher_;
    image_transport::Subscriber subscriber_;

    sensor_msgs::ImagePtr pub_msg_;

    int             flag_;
    std::string     METHOD_;

    /* for fisheye */

    std::string     fisheye_camera_info_url_;
    cv::Mat         K_;
    cv::Mat         D_;
    bool            getCAMINFO();
    double          balance_;
    bool            calib_fisheye();
    bool            camInfoStatus;


    cv::Mat sub_img_, pub_img_, process_img_;
    bool sub_img_status_, pub_img_status_;

    std::string     sub_topic_,
                    pub_topic_;


    int             framerate_,
                    image_width_,
                    image_height_;

    bool            isInit_,
                    isSub;
        int				**LUT_h, **LUT_w;
    double          fh;
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
    bool take_and_process_and_send_msg();
        bool getLUT();
    bool spin();
    bool calib();
    HeadCamNode();
    ~HeadCamNode();
};
}

HeadCam::HeadCamNode::HeadCamNode() :
    sub_img_status_(false), pub_img_status_(false), node_("~"), LUT_h(NULL), LUT_w(NULL), isInit_(false), isSub(false), camInfoStatus(false) {

    image_transport::ImageTransport it_(node_);

    node_.param("sub_topic", sub_topic_, std::string("/stereo/left/image_raw"));
    node_.param("pub_topic", pub_topic_, std::string("/calib/image_raw"));
    node_.param("framerate", framerate_, 30);
    node_.param("width", image_width_, 640);
    node_.param("height", image_height_, 480);
    node_.param("LUT_PARAM", fh, 0.000008205);
    node_.param("camera_info_url", fisheye_camera_info_url_, std::string("~/.ros/camera_info/camera.yml"));
    node_.param("method", flag_, 0);
    node_.param("balance", balance_, 1.0);


    publisher_  = it_.advertise(pub_topic_, 10);
    subscriber_ = it_.subscribe(sub_topic_, 10, &HeadCam::HeadCamNode::imageCallBack, this);


    sub_img_ = cv::Mat::zeros(image_height_, image_width_, 16);
    pub_img_ = cv::Mat::zeros(image_height_, image_width_, 16);
    process_img_ = cv::Mat::zeros(image_height_, image_width_, 16);

        /* two dimension */
        LUT_w = new int*[image_height_];
        LUT_h = new int*[image_height_];

        for (int i = 0; i < image_height_; i++) {
                LUT_w[i] = new int[image_width_];
                LUT_h[i] = new int[image_width_];

                memset(LUT_w[i], 0, sizeof(int)*image_width_);
                memset(LUT_h[i], 0, sizeof(int)*image_width_);

        }

        getLUT();
        getCAMINFO();
    isInit_ = true;

}


bool HeadCam::HeadCamNode::calib() {

    if (isSub) {
        cv::Mat temp = sub_img_.clone();
        for (int h = 0; h < image_height_; h++) {
            for (int w = 0; w < image_width_; w++) {
                if (LUT_h[h][w] > 0 && LUT_h[h][w] < image_height_ && LUT_w[h][w] > 0 && LUT_w[h][w] < image_width_)
                for (int i = 0; i < 3; i++) {

                        process_img_.at<cv::Vec3b>(LUT_h[h][w]  , LUT_w[h][w]  )[i] = temp.at<cv::Vec3b>(h, w)[i];
                        process_img_.at<cv::Vec3b>(LUT_h[h][w]+1, LUT_w[h][w]  )[i] = temp.at<cv::Vec3b>(h, w)[i];
                        process_img_.at<cv::Vec3b>(LUT_h[h][w]-1, LUT_w[h][w]  )[i] = temp.at<cv::Vec3b>(h, w)[i];
                        process_img_.at<cv::Vec3b>(LUT_h[h][w]  , LUT_w[h][w]+1)[i] = temp.at<cv::Vec3b>(h, w)[i];
                        process_img_.at<cv::Vec3b>(LUT_h[h][w]  , LUT_w[h][w]-1)[i] = temp.at<cv::Vec3b>(h, w)[i];

                }
            }
        }
        isSub = false;
        return true;
    } else {
        return false;
    }
//    int hw = image_width_ / 2,
//        hh = image_height_ / 2;
//ROS_WARN("kwp2000");

//    int new_h = 0, new_w = 0;

//    if (isSub) {
//        cv::Mat temp = sub_img_.clone();



//        for (int h = 0; h < image_height_; h++) {
//                for (int w = 0; w < image_width_; w++) {

//                        new_w = w;

//                        if (h >= hh) {
//                                new_h = h + floor(fh*abs(hh - h)*(pow(abs(hw - w), 2)));
//                        }
//                        else {
//                                new_h = h - floor(fh*abs(hh - h)*(pow(abs(hw - w), 2)));
//                        }

//                if ((new_h < image_height_) && (new_h > 1) && (new_w < image_width_) && (new_w > 1)) {
//                    for (int i = 0; i < 3; i++) {

//                            process_img_.at<cv::Vec3b>(new_h, new_w)[i] = temp.at<cv::Vec3b>(h, w)[i];
//                            process_img_.at<cv::Vec3b>(new_h+1, new_w)[i] = temp.at<cv::Vec3b>(h, w)[i];
//                            process_img_.at<cv::Vec3b>(new_h-1, new_w)[i] = temp.at<cv::Vec3b>(h, w)[i];
//                            process_img_.at<cv::Vec3b>(new_h, new_w+1)[i] = temp.at<cv::Vec3b>(h, w)[i];
//                            process_img_.at<cv::Vec3b>(new_h, new_w-1)[i] = temp.at<cv::Vec3b>(h, w)[i];

//                    }

//                }
//            }
//        }

//        isSub = false;
//        return true;
//    } else {
//        return false;
//    }

}

bool HeadCam::HeadCamNode::getLUT() {


        int hw = image_width_ / 2,
            hh = image_height_ / 2;


        int new_h = 0, new_w = 0;

        for (int h = 0; h < image_height_; h++) {
                for (int w = 0; w < image_width_; w++) {

                        new_w = w;

                        if (h >= hh) {
                                new_h = h + floor(fh*abs(hh - h)*(pow(abs(hw - w), 2)));
                        }
                        else {
                                new_h = h - floor(fh*abs(hh - h)*(pow(abs(hw - w), 2)));
                        }

            if ((new_h < image_height_) && (new_h > 1) && (new_w < image_width_) && (new_w > 1)) {


                                LUT_h[h][w] = new_h;
                                LUT_w[h][w] = new_w;

                        }
                }
        }

        return true;
}


HeadCam::HeadCamNode::~HeadCamNode() {


        for (int i = 0; i < image_height_; i++) {
                delete[] LUT_w[i];
                delete[] LUT_h[i];
        }

        delete LUT_w;
        delete LUT_h;

}

void HeadCam::HeadCamNode::imageCallBack(const sensor_msgs::ImageConstPtr& msg) {

    if (isInit_) {
        try {
            if (!isSub) {
                sub_img_ = cv_bridge::toCvShare(msg, "bgr8")->image;
                isSub = true;
            }

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Fail to sub_image");
        }
    }

}



bool HeadCam::HeadCamNode::getCAMINFO() {

    cv::FileStorage fs;
    cv::Matx33d     K;
    cv::Mat         D;

    if (fs.open(fisheye_camera_info_url_, cv::FileStorage::READ) ) {

        fs["camera_matrix"] >> K_;
        fs["distortion_coefficients"] >> D_;

        fs.release();
        camInfoStatus = true;

        return true;
    } else {
        return false;
    }

}

bool HeadCam::HeadCamNode::calib_fisheye() {

    if (isSub) {
        cv::Mat temp = sub_img_.clone();

        cv::Matx33d newK = K_;

        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_, D_, temp.size(), cv::noArray(), newK, balance_);
        cv::fisheye::undistortImage(temp, process_img_, K_, D_, newK);

        isSub = false;
        return true;
    } else {
        return false;
    }

}

bool HeadCam::HeadCamNode::take_and_process_and_send_msg() {

    if(!sub_img_.empty()) {
        if (isSub) {

            if (flag_ == KWP) {
                if (!calib()) ROS_WARN("Process Fail");
            } else if (flag_ == FISHEYE && camInfoStatus) {
                if (!calib_fisheye()) ROS_WARN("Process Fail");
            } else {
                ROS_ERROR("No Matching flag");
                return false;
            }

        }
        if (!process_img_.empty()) {
            pub_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", process_img_).toImageMsg();
            publisher_.publish(pub_msg_);
        }
        isSub = false;
        return true;
    } else {
        ROS_WARN("Empty Image");
        return false;
    }
}


bool HeadCam::HeadCamNode::spin() {

    ros::Rate loop_rate(this->framerate_);
    while(node_.ok()) {
        if (!take_and_process_and_send_msg()) {
            ROS_WARN("FAIL TO PROCESS");
        }
        ros::spinOnce();
        loop_rate.sleep();

    }

    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "head_cam");

    HeadCam::HeadCamNode hn;


    hn.spin();

    return EXIT_SUCCESS;
}
