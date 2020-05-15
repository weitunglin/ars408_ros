#include <string>
#include <map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include "std_msgs/String.h"
#include "ars408_ros/ARS408_CAN.h"
#include "ars408_msg/Test.h"
#include "ars408_msg/Tests.h"
#include "ars408_srv/Filter.h"

float RCS_filter = 0;
class visDriver
{
    public:
        visDriver();

    private:
        ros::NodeHandle node_handle;

        ros::Subscriber ars408rviz_sub;
        ros::Publisher markerArr_pub;
        ros::Publisher bbox_pub[8];
        std::map<int, ros::Subscriber> ars408_info_subs;
        std::map<int, ros::Publisher> overlayText_pubs;
        ros::ServiceServer filter_service;

        void ars408rviz_callback(const ars408_msg::Tests::ConstPtr& msg);
        void text_callback(const std_msgs::String::ConstPtr& msg, int id);
        bool set_filter(ars408_srv::Filter::Request &req, ars408_srv::Filter::Response &res);
};

visDriver::visDriver()
{
    node_handle = ros::NodeHandle("~");

    ars408rviz_sub = node_handle.subscribe("/testRects", 10, &visDriver::ars408rviz_callback, this);

    markerArr_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/markersArr", 10);
    bbox_pub[0] = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/point", 10);
    bbox_pub[1] = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/car", 10);
    bbox_pub[2] = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/truck", 10);
    bbox_pub[3] = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/reserved", 10);
    bbox_pub[4] = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/motorcycle", 10);
    bbox_pub[5] = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bicycle", 10);
    bbox_pub[6] = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/wide", 10);
    bbox_pub[7] = node_handle.advertise<jsk_recognition_msgs::BoundingBoxArray>("/other", 10);

    ars408_info_subs[0x201] = node_handle.subscribe<std_msgs::String>("/info_201", 10, boost::bind(&visDriver::text_callback, this, _1, 0x201));
    ars408_info_subs[0x700] = node_handle.subscribe<std_msgs::String>("/info_700", 10, boost::bind(&visDriver::text_callback, this, _1, 0x700));
    ars408_info_subs[0x600] = node_handle.subscribe<std_msgs::String>("/info_clu_sta", 10, boost::bind(&visDriver::text_callback, this, _1, 0x600));
    ars408_info_subs[0x60A] = node_handle.subscribe<std_msgs::String>("/info_obj_sta", 10, boost::bind(&visDriver::text_callback, this, _1, 0x60A));

    overlayText_pubs[0x201] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText201", 10);
    overlayText_pubs[0x700] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText700", 10);
    overlayText_pubs[0x600] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText600", 10);
    overlayText_pubs[0x60A] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText60A", 10);

    filter_service = node_handle.advertiseService("/filter", &visDriver::set_filter, this);
}

void visDriver::text_callback(const std_msgs::String::ConstPtr& msg, int id)
{
    jsk_rviz_plugins::OverlayText overlaytext;
    overlaytext.action = jsk_rviz_plugins::OverlayText::ADD;
    overlaytext.text = msg->data;
    overlayText_pubs[id].publish(overlaytext);
}

void visDriver::ars408rviz_callback(const ars408_msg::Tests::ConstPtr& msg)
{
    visualization_msgs::MarkerArray marArr;
    jsk_recognition_msgs::BoundingBoxArray bboxArr[8];

    for (auto it = msg->tests.begin(); it != msg->tests.end(); ++it)
    {
        // Rect
        visualization_msgs::Marker marker_rect;
        jsk_recognition_msgs::BoundingBox bbox;

        marker_rect.header.frame_id = "/my_frame";
        marker_rect.header.stamp = ros::Time::now();

        marker_rect.ns = "rect";
        marker_rect.id = it->id;
        marker_rect.type = visualization_msgs::Marker::CUBE;
        marker_rect.action = visualization_msgs::Marker::ADD;
        marker_rect.pose.position.x = it->x;
        marker_rect.pose.position.y = it->y;
        marker_rect.pose.position.z = 0.05;
        marker_rect.pose.orientation.x = 0.0;
        marker_rect.pose.orientation.y = 0.0;
        marker_rect.pose.orientation.z = (it->angle)/90;
        marker_rect.pose.orientation.w = 1.0;
        marker_rect.scale.x = it->height;
        marker_rect.scale.y = it->width;
        marker_rect.scale.z = 0.1;

        bbox.header.frame_id = "/my_frame";
        bbox.header.stamp = ros::Time::now();
        bbox.dimensions.x = marker_rect.scale.x;
        bbox.dimensions.y = marker_rect.scale.y;
        bbox.dimensions.z = marker_rect.scale.z;
        bbox.pose.orientation.x = marker_rect.pose.orientation.x;
        bbox.pose.orientation.y = marker_rect.pose.orientation.y;
        bbox.pose.orientation.z = marker_rect.pose.orientation.z;
        bbox.pose.orientation.w = marker_rect.pose.orientation.w;
        bbox.pose.position.x = marker_rect.pose.position.x;
        bbox.pose.position.y = marker_rect.pose.position.y;
        bbox.pose.position.z = marker_rect.pose.position.z;

        if (it->classT == 0x00)
        {
            bboxArr[0].boxes.push_back(bbox);
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x01)
        {
            bboxArr[1].boxes.push_back(bbox);
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x02)
        {
            bboxArr[2].boxes.push_back(bbox);
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x04)
        {
            bboxArr[4].boxes.push_back(bbox);
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x05)
        {
            bboxArr[5].boxes.push_back(bbox);
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x06)
        {
            bboxArr[6].boxes.push_back(bbox);
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x07 || it->classT==0x03)
        {
            bboxArr[3].boxes.push_back(bbox);
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }
        else
        {
            bboxArr[7].boxes.push_back(bbox);
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }

        marker_rect.lifetime = ros::Duration(0.1);

        // Text
        visualization_msgs::Marker marker_text;
        marker_text.header.frame_id = "/my_frame";
        marker_text.header.stamp = ros::Time::now();

        marker_text.ns = "text";
        marker_text.id = it->id;

        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::Marker::ADD;
        // marker_text.text = it->strs;
         std::stringstream ss;
        ss << "DynProp: " <<it->dynProp << std::endl;
        ss << "RCS: " << it->RCS << std::endl;
        // ss << "Prob: " << it->prob << std::endl;
        marker_text.text = ss.str();

        marker_text.pose.position.x = it->x;
        marker_text.pose.position.y = it->y;
        marker_text.pose.position.z = 0.4;
        marker_text.pose.orientation.x = 0.0;
        marker_text.pose.orientation.y = 0.0;
        marker_text.pose.orientation.z = 0.0;
        marker_text.pose.orientation.w = 1.0;
        marker_text.scale.z = 0.5;

        marker_text.color.r = 1.0f;
        marker_text.color.g = 1.0f;
        marker_text.color.b = 1.0f;
        marker_text.color.a = 1.0;

        marker_text.lifetime = ros::Duration(0.1);
        if(it->RCS > RCS_filter){
        marArr.markers.push_back(marker_rect);
        marArr.markers.push_back(marker_text);
        }
    }

    if (msg->tests.size() > 0){
        markerArr_pub.publish(marArr);
        for(int i = 0; i < 8; i++){
            bboxArr[i].header.frame_id = "/my_frame";
            bboxArr[i].header.stamp = ros::Time::now();
            bbox_pub[i].publish(bboxArr[i]);
            ros::spinOnce();
        }
    }
}

bool visDriver::set_filter(ars408_srv::Filter::Request  &req, ars408_srv::Filter::Response &res)
{
    res.RCS_filter = req.RCS_filter;
    RCS_filter = res.RCS_filter;
    std::cout<< "server response : " << res.RCS_filter << std::endl;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualRadar");
    visDriver node;
    ros::Rate r(60);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
