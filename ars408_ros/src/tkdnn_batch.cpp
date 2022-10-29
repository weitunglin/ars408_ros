#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <tkDNN/Yolo3Detection.h>
#include <cv_bridge/cv_bridge.h>

class TKDNN {
public:
    ros::NodeHandle nh;

    tk::dnn::Yolo3Detection yolo;
    tk::dnn::DetectionNN *detNN;

    std::string rt_file;
    int num_classes;
    int batch_size;
    float conf_thres;

    std::vector<std::string> names{"front_center", "front_left", "front_right", "rear_center", "rear_left", "rear_right"};
    // std::vector<std::string> names{"front_center"};
    std::vector<message_filters::Subscriber<sensor_msgs::Image>> sub;
    message_filters::Subscriber<sensor_msgs::Image> sub0;
    message_filters::Subscriber<sensor_msgs::Image> sub1;
    message_filters::Subscriber<sensor_msgs::Image> sub2;
    message_filters::Subscriber<sensor_msgs::Image> sub3;
    message_filters::Subscriber<sensor_msgs::Image> sub4;
    message_filters::Subscriber<sensor_msgs::Image> sub5;
    std::map<std::string, ros::Publisher> pub;

    ros::Subscriber subx;
    ros::Publisher pubx;

    std::vector<cv::Mat> batch_frame;
    std::vector<cv::Mat> batch_dnn_input;

    void batch_callback(sensor_msgs::Image img1, sensor_msgs::Image img2, sensor_msgs::Image img3,
        sensor_msgs::Image img4, sensor_msgs::Image img5, sensor_msgs::Image img6);
    void callback(sensor_msgs::Image img1);

    float avg_time = 0;

    TKDNN(ros::NodeHandle& n) : nh(n) {
        // TODO
        // subscribes 6 calibrated images using Synchronizer
        // publishes 6 bounding_boxes

        nh.param<std::string>("rt_file", rt_file, "/rt_file/yolo4_fp16.rt");
        nh.param<int>("num_classes", num_classes, 80);
        nh.param<int>("batch_size", batch_size, 6);
        nh.param<float>("conf_thres", conf_thres, 0.6);

        std::string path = ros::package::getPath("ars408_ros");
        detNN = &yolo;
        detNN->init(path+rt_file, num_classes, batch_size, conf_thres);

        for (auto i: names) {
            ROS_INFO_STREAM("/rgb/"+i+"/calib_image");
            pub[i] = nh.advertise<sensor_msgs::Image>("/rgb/"+i+"/detected_output", 2);

            // single
            subx = nh.subscribe("/rgb/"+i+"/calib_image", 1, &TKDNN::callback, this);
            pubx = nh.advertise<sensor_msgs::Image>("/rgb/"+i+"/detected_output", 2);
        }
        
        /*
        sub0.subscribe(nh, "/rgb/front_center/calib_image", 10);
        sub1.subscribe(nh, "/rgb/front_left/calib_image", 10);
        sub2.subscribe(nh, "/rgb/front_right/calib_image", 10);
        sub3.subscribe(nh, "/rgb/rear_center/calib_image", 10);
        sub4.subscribe(nh, "/rgb/rear_right/calib_image", 10);
        sub5.subscribe(nh, "/rgb/rear_left/calib_image", 10);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), sub0, sub1, sub2, sub3, sub4, sub5);
        sync.registerCallback(&TKDNN::batch_callback, this);
        */
    }

};

void TKDNN::callback(sensor_msgs::Image img1) {
    // ROS_INFO_STREAM("callback");

    // t0
    ros::Time now();
    std::vector<sensor_msgs::Image> msgs{img1, img1, img1, img1, img1, img1};

    batch_frame.clear();
    batch_dnn_input.clear();

    for (auto i: msgs) {
        i.encoding = sensor_msgs::image_encodings::BGR8;
        cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(i, sensor_msgs::image_encodings::BGR8);
        batch_frame.push_back(img_ptr->image.clone());
        batch_dnn_input.push_back(img_ptr->image.clone());
    }

    // t1
    detNN->update(batch_dnn_input, batch_size);

    // t2
    detNN->draw(batch_frame);

    // t3
    cv_bridge::CvImage bridge_img;
    sensor_msgs::Image msg_img;
    std_msgs::Header header;
    for (uint i = 0; i < batch_frame.size(); ++i) {
        header.stamp = ros::Time::now();
        bridge_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, batch_frame[i]);
        bridge_img.toImageMsg(msg_img);
        pub[names[i]].publish(msg_img);
    }
}

void TKDNN::batch_callback(sensor_msgs::Image img1, sensor_msgs::Image img2, sensor_msgs::Image img3,
    sensor_msgs::Image img4, sensor_msgs::Image img5, sensor_msgs::Image img6) {
    ROS_INFO_STREAM("callback");

    // t0
    std::vector<sensor_msgs::Image> msgs{img1, img2, img3, img4, img5, img6};

    batch_frame.clear();
    batch_dnn_input.clear();

    for (auto i: msgs) {
        cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(i, sensor_msgs::image_encodings::BGR8);
        batch_frame.push_back(img_ptr->image.clone());
        batch_dnn_input.push_back(img_ptr->image.clone());
    }

    // t1
    detNN->update(batch_dnn_input, batch_size);

    // t2
    detNN->draw(batch_frame);

    // t3
    cv_bridge::CvImage bridge_img;
    sensor_msgs::Image msg_img;
    std_msgs::Header header;
    for (uint i = 0; i < batch_frame.size(); ++i) {
        header.stamp = ros::Time::now();
        bridge_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, batch_frame[i]);
        bridge_img.toImageMsg(msg_img);
        pub[names[i]].publish(msg_img);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tKDNN Batch Node");

    ros::NodeHandle n;

    TKDNN tkdnn(n);

    ros::spin();

    return 0;
}
