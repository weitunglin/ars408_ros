#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <tkDNN/Yolo3Detection.h>
#include <cv_bridge/cv_bridge.h>

#include <ars408_msg/BatchImage.h>

class TKDNN {
public:
    ros::NodeHandle nh;

    tk::dnn::Yolo3Detection yolo;
    tk::dnn::DetectionNN *detNN;

    std::string rt_file;
    int num_classes;
    int batch_size;
    float conf_thres;

    std::vector<std::string> names{"front_left", "front_center", "front_right", "rear_left", "rear_center", "rear_right"};
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
    void callback(ars408_msg::BatchImage batch_image);

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

        subx = nh.subscribe("/rgb/synced_batch_image", 1, &TKDNN::callback, this);
        for (auto i: names) {
            pub[i] = nh.advertise<sensor_msgs::Image>("/rgb/"+i+"/detected_output", 1);
        }
    }

};

void TKDNN::callback(ars408_msg::BatchImage batch_image) {
    // t0
    ros::Time t0 = ros::Time::now();
    std::vector<sensor_msgs::Image> msgs {
        batch_image.front_left,
        batch_image.front_center,
        batch_image.front_right,
        batch_image.rear_left,
        batch_image.rear_center,
        batch_image.rear_right,
    };

    batch_frame.clear();
    batch_dnn_input.clear();

    for (auto i: msgs) {
        i.encoding = sensor_msgs::image_encodings::BGR8;
        cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(i, sensor_msgs::image_encodings::BGR8);
        batch_frame.push_back(img_ptr->image.clone());
        batch_dnn_input.push_back(img_ptr->image.clone());
    }

    // t1
    ros::Time t1 = ros::Time::now();
    detNN->update(batch_dnn_input, batch_size);

    // t2
    ros::Time t2 = ros::Time::now();
    detNN->draw(batch_frame);

    // t3
    ros::Time t3 = ros::Time::now();
    cv_bridge::CvImage bridge_img;
    sensor_msgs::Image msg_img;
    std_msgs::Header header;
    for (uint i = 0; i < batch_frame.size(); ++i) {
        header.stamp = ros::Time::now();
        bridge_img = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, batch_frame[i]);
        bridge_img.toImageMsg(msg_img);
        pub[names[i]].publish(msg_img);
    }
    
    // t4
    ros::Time t4 = ros::Time::now();

    ROS_INFO_STREAM_THROTTLE(3, "-------------------------");
    ROS_INFO_STREAM_THROTTLE(3, "pre time\t" << (t1 - t0).toSec());
    ROS_INFO_STREAM_THROTTLE(3, "model time\t" << (t2 - t1).toSec());
    ROS_INFO_STREAM_THROTTLE(3, "draw time\t" << (t3 - t2).toSec());
    ROS_INFO_STREAM_THROTTLE(3, "post time\t" << (t4 - t3).toSec());
    ROS_INFO_STREAM_THROTTLE(3, "-------------------------");
}

void TKDNN::batch_callback(sensor_msgs::Image img1, sensor_msgs::Image img2, sensor_msgs::Image img3,
    sensor_msgs::Image img4, sensor_msgs::Image img5, sensor_msgs::Image img6) {
    ROS_INFO_STREAM("batch callback");

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
    ros::init(argc, argv, "tKDNN_Batch_Node");

    ros::NodeHandle n;

    TKDNN tkdnn(n);

    ros::spin();

    return 0;
}
