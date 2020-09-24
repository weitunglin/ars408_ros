#include <string>
#include <map>
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "ars408_ros/ARS408_CAN.h"
#include "ars408_msg/RadarPoint.h"
#include "ars408_msg/RadarPoints.h"
#include "ars408_msg/pathPoint.h"
#include "ars408_msg/pathPoints.h"
#include "ars408_srv/Filter.h"

// #define RVIZ_ARROW
// #define RVIZ_TEXT
#define PI 3.14159265

float nowSpeed = 0;
float RCS_filter = -10000;
float predict_speed = 0;
float predict_zaxis = 0;

class visDriver
{
    public:
        visDriver();

    private:
        std::map<float, float> predictPoints;
        ros::NodeHandle node_handle;

        ros::Subscriber ars408rviz_sub;
        ros::Publisher markerArr_pub;
        ros::Publisher predict_pub;
        ros::Publisher pathPoints_pub;
        std::map<int, ros::Subscriber> ars408_info_subs;
        std::map<int, ros::Subscriber> motion_info_subs;
        std::map<int, ros::Publisher> overlayText_pubs;
        ros::ServiceServer filter_service;

        void ars408rviz_callback(const ars408_msg::RadarPoints::ConstPtr& msg);
        void text_callback(const std_msgs::String::ConstPtr& msg, int id);
        void text_callback_float(const std_msgs::Float32::ConstPtr& msg, std::string topicName, int id);
        bool set_filter(ars408_srv::Filter::Request &req, ars408_srv::Filter::Response &res);
};

visDriver::visDriver()
{
    node_handle = ros::NodeHandle("~");

    ars408rviz_sub = node_handle.subscribe("/radarPub", 1, &visDriver::ars408rviz_callback, this);

    markerArr_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/markersArr", 1);
    predict_pub = node_handle.advertise<nav_msgs::Path>("/predictPath", 1);
    pathPoints_pub = node_handle.advertise<ars408_msg::pathPoints>("/pathPoints", 1);

    ars408_info_subs[0x201] = node_handle.subscribe<std_msgs::String>("/info_201", 1, boost::bind(&visDriver::text_callback, this, _1, 0x201));
    ars408_info_subs[0x700] = node_handle.subscribe<std_msgs::String>("/info_700", 1, boost::bind(&visDriver::text_callback, this, _1, 0x700));
    ars408_info_subs[0x600] = node_handle.subscribe<std_msgs::String>("/info_clu_sta", 1, boost::bind(&visDriver::text_callback, this, _1, 0x600));
    ars408_info_subs[0x60A] = node_handle.subscribe<std_msgs::String>("/info_obj_sta", 1, boost::bind(&visDriver::text_callback, this, _1, 0x60A));

    motion_info_subs[0x300] = node_handle.subscribe<std_msgs::Float32>("/speed", 1, boost::bind(&visDriver::text_callback_float, this, _1, "Speed", 0x300));
    motion_info_subs[0x301] = node_handle.subscribe<std_msgs::Float32>("/zaxis", 1, boost::bind(&visDriver::text_callback_float, this, _1, "ZAxis", 0x301));
    motion_info_subs[0x302] = node_handle.subscribe<std_msgs::Float32>("/zaxisFilter", 1, boost::bind(&visDriver::text_callback_float, this, _1, "zaxisFilter", 0x302));
    motion_info_subs[0x303] = node_handle.subscribe<std_msgs::Float32>("/zaxisKalman", 1, boost::bind(&visDriver::text_callback_float, this, _1, "zaxisKalman", 0x303));

    overlayText_pubs[0x201] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText201", 1);
    overlayText_pubs[0x700] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText700", 1);
    overlayText_pubs[0x600] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText600", 1);
    overlayText_pubs[0x60A] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText60A", 1);
    overlayText_pubs[0x300] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText300", 1);
    overlayText_pubs[0x301] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText301", 1);
    overlayText_pubs[0x302] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText302", 1);
    overlayText_pubs[0x303] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText303", 1);

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

    if (topicName == "Speed")
    {
        nowSpeed = (int)(msg->data / 2.5) * 2.5;
        predict_speed = msg->data * 4;
        predict_speed /= 50;
    }
    if (topicName == "ZAxis")
    {
        predict_zaxis=msg->data* 4;
        predict_zaxis /= 50;
    }
    visualization_msgs::Marker marker_predict;

    nav_msgs::Path predict_path;
    predict_path.header.frame_id = "/my_frame";
    predict_path.header.stamp = ros::Time::now();

    ars408_msg::pathPoints pathPs;

    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    
    for(uint32_t i = 0; i <= 50; i++)
    {
        geometry_msgs::PoseStamped ps;
        geometry_msgs::Point p;
        
        p.z = -1;

        x1 = cos((90-predict_zaxis * i) * PI / 180) * predict_speed + x0;
        y1 = sin((90-predict_zaxis * i) * PI / 180) * predict_speed + y0;

        if(i == 0){
            x1 = 0;
            y1 = 0;
        }

        p.x = y1;
        p.y = x1;

        x0 = x1;
        y0 = y1;

        ars408_msg::pathPoint temp;
        temp.X = p.x;
        temp.Y = p.y;
        pathPs.pathPoints.push_back(temp);

        ps.pose.position = p;
        predict_path.poses.push_back(ps);
    }
    pathPoints_pub.publish(pathPs);
    predict_pub.publish(predict_path);
}

void visDriver::ars408rviz_callback(const ars408_msg::RadarPoints::ConstPtr& msg)
{
    visualization_msgs::MarkerArray marArr;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Clear
    marker.action = visualization_msgs::Marker::DELETEALL;
    marArr.markers.push_back(marker);
    markerArr_pub.publish(marArr);

    for (auto it = msg->rps.begin(); it != msg->rps.end(); ++it)
    {
        // Rect
        visualization_msgs::Marker marker_rect;

        marker_rect.header.frame_id = "/my_frame";
        marker_rect.header.stamp = ros::Time::now();

        marker_rect.ns = "rect";
        marker_rect.id = it->id;
        marker_rect.type = visualization_msgs::Marker::SPHERE;
        marker_rect.action = visualization_msgs::Marker::ADD;
        marker_rect.pose.position.x = it->distX;
        marker_rect.pose.position.y = it->distY;
        marker_rect.pose.position.z = 0.05;
        // marker_rect.scale.x = it->height;
        // marker_rect.scale.y = it->width;
        marker_rect.scale.x = 1;
        marker_rect.scale.y = 1;
        marker_rect.scale.z = 0.1;

        double theta = it->angle / 180.0 * M_PI;
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

        if (it->isDanger)
        {
            // gray: Danger
            marker_rect.ns = "Danger";
            marker_rect.color.r = 0.7f;
            marker_rect.color.g = 0.7f;
            marker_rect.color.b = 0.7f;
            marker_rect.color.a = 1.0f;
        }

        #ifdef RVIZ_TEXT
        // Text
        visualization_msgs::Marker marker_text;
        marker_text.header.frame_id = "/my_frame";
        marker_text.header.stamp = ros::Time::now();

        marker_text.ns = "text";
        marker_text.id = it->id;

        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::Marker::ADD;

        std::stringstream ss;
        ss << "DynProp: " << ARS408::DynProp[it->dynProp] << std::endl;
        ss << "RCS: " << it->rcs << std::endl;
        ss << "VrelLong: " << it->vrelX << std::endl;
        ss << "VrelLat: " << it->vrelY << std::endl;
        ss << "Distance: " << sqrt(pow(it->distX, 2) + pow(it->distY, 2)) << std::endl;

        marker_text.text = ss.str();

        marker_text.pose.position.x = it->distX;
        marker_text.pose.position.y = it->distY;
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
        #endif

        #ifdef RVIZ_ARROW
        //Arrow
        visualization_msgs::Marker marker_arrow;

        if (it->dynProp != 1) // not stationary
        {
            marker_arrow.header.frame_id = "/my_frame";
            marker_arrow.header.stamp = ros::Time::now();

            marker_arrow.ns = "arrow";
            marker_arrow.id = it->id;
            marker_arrow.type = visualization_msgs::Marker::ARROW;
            marker_arrow.action = visualization_msgs::Marker::ADD;

            float rpSpeed = hypot(nowSpeed + it->vrelX, it->vrelY);

            marker_arrow.pose.position.x = it->distX;
            marker_arrow.pose.position.y = it->distY;
            marker_arrow.pose.position.z = 0.2;
            marker_arrow.scale.x = abs(rpSpeed);
            marker_arrow.scale.y = 0.2;
            marker_arrow.scale.z = 0.1;

            marker_arrow.color.r = 1.0f;
            marker_arrow.color.g = 1.0f;
            marker_arrow.color.b = 1.0f;
            marker_arrow.color.a = 1.0f;

            double arrowDir = atan2(it->vrelY, nowSpeed + it->vrelX);

            marker_arrow.pose.orientation.x = 0.0 * sin(arrowDir/2.0);
            marker_arrow.pose.orientation.y = 0.0 * sin(arrowDir/2.0);
            marker_arrow.pose.orientation.z = 1.0 * sin(arrowDir/2.0);
            marker_arrow.pose.orientation.w = cos(arrowDir/2.0);

            if (it->rcs > RCS_filter && rpSpeed > 0.1) {
                marArr.markers.push_back(marker_arrow);
            }
        }
        #endif

        if (it->rcs > RCS_filter) {
            marArr.markers.push_back(marker_rect);
            #ifdef RVIZ_TEXT
            marArr.markers.push_back(marker_text);
            #endif
        }
    }

    if (msg->rps.size() > 0) {
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
