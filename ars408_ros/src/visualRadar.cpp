#include <string>
#include <map>
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "ars408_ros/ARS408_CAN.h"
#include "ars408_msg/Test.h"
#include "ars408_msg/Tests.h"
#include "ars408_srv/Filter.h"

float RCS_filter = -10000;
class visDriver
{
    public:
        visDriver();

    private:
        ros::NodeHandle node_handle;

        ros::Subscriber ars408rviz_sub;
        ros::Publisher markerArr_pub;
        std::map<int, ros::Subscriber> ars408_info_subs;
        std::map<int, ros::Subscriber> motion_info_subs;
        std::map<int, ros::Publisher> overlayText_pubs;
        ros::ServiceServer filter_service;

        void ars408rviz_callback(const ars408_msg::Tests::ConstPtr& msg);
        void text_callback(const std_msgs::String::ConstPtr& msg, int id);
        void text_callback_float(const std_msgs::Float32::ConstPtr& msg, std::string topicName, int id);
        bool set_filter(ars408_srv::Filter::Request &req, ars408_srv::Filter::Response &res);
};

visDriver::visDriver()
{
    node_handle = ros::NodeHandle("~");

    ars408rviz_sub = node_handle.subscribe("/testRects", 10, &visDriver::ars408rviz_callback, this);

    markerArr_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/markersArr", 10);

    ars408_info_subs[0x201] = node_handle.subscribe<std_msgs::String>("/info_201", 10, boost::bind(&visDriver::text_callback, this, _1, 0x201));
    ars408_info_subs[0x700] = node_handle.subscribe<std_msgs::String>("/info_700", 10, boost::bind(&visDriver::text_callback, this, _1, 0x700));
    ars408_info_subs[0x600] = node_handle.subscribe<std_msgs::String>("/info_clu_sta", 10, boost::bind(&visDriver::text_callback, this, _1, 0x600));
    ars408_info_subs[0x60A] = node_handle.subscribe<std_msgs::String>("/info_obj_sta", 10, boost::bind(&visDriver::text_callback, this, _1, 0x60A));

    motion_info_subs[0x300] = node_handle.subscribe<std_msgs::Float32>("/speed", 10, boost::bind(&visDriver::text_callback_float, this, _1, "Speed", 0x300));
    motion_info_subs[0x301] = node_handle.subscribe<std_msgs::Float32>("/zaxis", 10, boost::bind(&visDriver::text_callback_float, this, _1, "ZAxis", 0x301));
    motion_info_subs[0x302] = node_handle.subscribe<std_msgs::Float32>("/zaxisFilter", 10, boost::bind(&visDriver::text_callback_float, this, _1, "zaxisFilter", 0x302));
    motion_info_subs[0x303] = node_handle.subscribe<std_msgs::Float32>("/zaxisKalman", 10, boost::bind(&visDriver::text_callback_float, this, _1, "zaxisKalman", 0x303));

    overlayText_pubs[0x201] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText201", 10);
    overlayText_pubs[0x700] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText700", 10);
    overlayText_pubs[0x600] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText600", 10);
    overlayText_pubs[0x60A] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText60A", 10);
    overlayText_pubs[0x300] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText300", 10);
    overlayText_pubs[0x301] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText301", 10);
    overlayText_pubs[0x302] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText302", 10);
    overlayText_pubs[0x303] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText303", 10);

    filter_service = node_handle.advertiseService("/filter", &visDriver::set_filter, this);
}

void visDriver::text_callback(const std_msgs::String::ConstPtr& msg, int id)
{
    jsk_rviz_plugins::OverlayText overlaytext;
    overlaytext.action = jsk_rviz_plugins::OverlayText::ADD;
    overlaytext.text = msg->data;
    overlayText_pubs[id].publish(overlaytext);
}

void visDriver::text_callback_float(const std_msgs::Float32::ConstPtr& msg, std::string topicName, int id)
{
    jsk_rviz_plugins::OverlayText overlaytext;
    overlaytext.action = jsk_rviz_plugins::OverlayText::ADD;
    std::stringstream ss;

    ss << topicName << ": " << msg->data << std::endl;
    overlaytext.text = ss.str();
    overlayText_pubs[id].publish(overlaytext);
}

void visDriver::ars408rviz_callback(const ars408_msg::Tests::ConstPtr& msg)
{
    visualization_msgs::MarkerArray marArr;

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    marker.action = visualization_msgs::Marker::DELETEALL;
    marArr.markers.push_back(marker);
    markerArr_pub.publish(marArr);

    for (auto it = msg->tests.begin(); it != msg->tests.end(); ++it)
    {
        // Rect
        visualization_msgs::Marker marker_rect;

        marker_rect.header.frame_id = "/my_frame";
        marker_rect.header.stamp = ros::Time::now();

        marker_rect.ns = "rect";
        marker_rect.id = it->id;
        marker_rect.type = visualization_msgs::Marker::SPHERE;
        marker_rect.action = visualization_msgs::Marker::ADD;
        marker_rect.pose.position.x = it->x;
        marker_rect.pose.position.y = it->y;
        marker_rect.pose.position.z = 0.05;
        // marker_rect.scale.x = it->height;
        // marker_rect.scale.y = it->width;
        marker_rect.scale.x = 1;
        marker_rect.scale.y = 1;
        marker_rect.scale.z = 0.1;

        double theta = it->angle / 360.0 * M_PI; 
        marker_rect.pose.orientation.x = 0.0 * sin(theta/2.0); 
        marker_rect.pose.orientation.y = 0.0 * sin(theta/2.0);
        marker_rect.pose.orientation.z = 1.0 * sin(theta/2.0);
        marker_rect.pose.orientation.w = cos(theta/2.0);

        if (it->classT == 0x00)
        {
            // White: point
            marker_rect.ns = "point";
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0f;
        }
        else if (it->classT == 0x01)
        {
            // Red: car
            marker_rect.ns = "car";
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0f;
        }
        else if (it->classT == 0x02)
        {
            // Purple： truck
            marker_rect.ns = "truck";
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0f;
        }
        else if (it->classT == 0x03 || it->classT==0x07)
        {
            // Blue: reserved
            marker_rect.ns = "reserved";
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0f;
        }
        else if (it->classT == 0x04)
        {
            // Yellow: motorcycle
            marker_rect.ns = "motorcycle";
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x05)
        {
            // Green: bicycle
            marker_rect.ns = "bicycle";
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0f;
        }
        else if (it->classT == 0x06)
        {
            // Cyan: wide
            marker_rect.ns = "wide";
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0f;
        }
        else
        {
            // Orange: others
            marker_rect.ns = "others";
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 0.5f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0f;
        }

        //Arrow
        visualization_msgs::Marker arrow;

        arrow.header.frame_id = "/my_frame";
        arrow.header.stamp = ros::Time::now();

        arrow.ns = "arrow";
        arrow.id = it->id;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        double speed = abs(it->VrelLong)/5;
        if(speed == 0)
            speed = abs(it->VrelLat)/5;
        if(speed < 1 && speed > 0)
            speed = 1;

        arrow.pose.position.x = it->x;
        arrow.pose.position.y = it->y;
        arrow.pose.position.z = 0.2;
        arrow.scale.x = speed;
        arrow.scale.y = 0.2;
        arrow.scale.z = 0.1;

        arrow.color.r = 1.0f;
        arrow.color.g = 1.0f;
        arrow.color.b = 1.0f;
        arrow.color.a = 1.0f;

        double x,y;

        if(abs(it->VrelLat) > abs(it->VrelLong)){
            if(it->VrelLat >= 0)
                x = it->VrelLat + 0.1, y = it->VrelLat;
            else if(it->VrelLat <= 0)
                x = -(it->VrelLat + 0.1), y = it->VrelLat;
        }
        else{
            if(it->VrelLong >=0 && it->VrelLat >= 0)       //左上
                x = it->VrelLong, y = it->VrelLat;
            else if(it->VrelLong <= 0 && it->VrelLat >= 0) //左下
                x = it->VrelLat, y = -it->VrelLong;
            else if(it->VrelLong >= 0 && it->VrelLat <= 0) //右上
                x = it->VrelLong, y = it->VrelLat;
            else if(it->VrelLong <= 0 && it->VrelLat <= 0) //右下
                x = -it->VrelLat, y = it->VrelLong;

            if(!(x == 0 && y == 0)){
                if(x == 0)
                    x = 0.01;
                if(y == 0)
                    y = 0.01;
            }
        }

        arrow.pose.orientation.x = x;
        arrow.pose.orientation.y = y;
        arrow.pose.orientation.z = 0.0;
        arrow.pose.orientation.w = 0.0;

        // Text
        visualization_msgs::Marker marker_text;
        marker_text.header.frame_id = "/my_frame";
        marker_text.header.stamp = ros::Time::now();

        marker_text.ns = "text";
        marker_text.id = it->id;
        marker_text.scale.x = 5;
        marker_text.scale.y = 5;

        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::Marker::ADD;

        std::stringstream ss;
        ss << "DynProp: " <<it->dynProp << std::endl;
        ss << "RCS: " << it->RCS << std::endl;
        ss << "VrelLong: " << it->VrelLong << std::endl;
        ss << "VrelLat: " << it->VrelLat << std::endl;
        ss << "Distance: " << sqrt(pow(it->x, 2) + pow(it->y, 2)) << std::endl;

        //  ss << "x: " << it->x << std::endl;
        // ss << "y: " << it->y << std::endl;
        // ss << "x: " << x << std::endl;
        // ss << "y: " << y << std::endl;
        // ss << "z: " << marker_rect.pose.orientation.z << std::endl;
        // ss << "w: " << marker_rect.pose.orientation.w << std::endl;
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

        // marker_text.lifetime = ros::Duration(0.1);
        if(it->RCS > RCS_filter){
            marArr.markers.push_back(marker_rect);
            marArr.markers.push_back(marker_text);
            marArr.markers.push_back(arrow);
        }
    }

    if (msg->tests.size() > 0) {
        markerArr_pub.publish(marArr);
    }
    ros::spinOnce();
}

bool visDriver::set_filter(ars408_srv::Filter::Request  &req, ars408_srv::Filter::Response &res)
{
    res.RCS_filter = req.RCS_filter;
    RCS_filter = res.RCS_filter;
    std::cout<< "server response: " << res.RCS_filter << std::endl;
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
