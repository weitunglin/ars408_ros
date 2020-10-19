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
#include "ars408_msg/GPSinfo.h"
#include "ars408_srv/Filter.h"

// #define RVIZ_ARROW
// #define RVIZ_TEXT
// #define RVIZ_TRAJECTORY
// #define RVIZ_RANGE
// #define RVIZ_RADARPOINTS_TRAJECTORY

float nowSpeed = 0;
float RCS_filter = -10000;
float predict_speed = 0;
float predict_zaxis = 0;

float init_long = -1;
float init_lat = -1;
ars408_msg::pathPoints gpsPoints;

float angle;

std::map<int, ars408_msg::pathPoints> radarPoints;

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
        ros::Publisher trajectory_pub;
        ros::Publisher radar_trajectory_pub;
        ros::Publisher world_trajectory_pub;
        ros::Publisher range_pub;

        std::map<int, ros::Subscriber> ars408_info_subs;
        std::map<int, ros::Subscriber> motion_info_subs;
        std::map<int, ros::Publisher> overlayText_pubs;
        ros::ServiceServer filter_service;

        void ars408rviz_callback(const ars408_msg::RadarPoints::ConstPtr& msg);
        void text_callback(const std_msgs::String::ConstPtr& msg, int id);
        void text_callback_float(const ars408_msg::GPSinfo::ConstPtr& msg, int id);
        bool set_filter(ars408_srv::Filter::Request &req, ars408_srv::Filter::Response &res);
};

visDriver::visDriver()
{
    node_handle = ros::NodeHandle("~");

    ars408rviz_sub = node_handle.subscribe("/radarPub", 1, &visDriver::ars408rviz_callback, this);

    markerArr_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/markersArr", 1);
    predict_pub = node_handle.advertise<nav_msgs::Path>("/predictPath", 1);
    pathPoints_pub = node_handle.advertise<ars408_msg::pathPoints>("/pathPoints", 1);
    world_trajectory_pub = node_handle.advertise<visualization_msgs::Marker>("/world_trajectory", 1);
    trajectory_pub = node_handle.advertise<visualization_msgs::Marker>("/trajectory", 1);
    radar_trajectory_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/radar_trajectory", 1);
    range_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/range", 1);

    ars408_info_subs[0x201] = node_handle.subscribe<std_msgs::String>("/info_201", 1, boost::bind(&visDriver::text_callback, this, _1, 0x201));
    ars408_info_subs[0x700] = node_handle.subscribe<std_msgs::String>("/info_700", 1, boost::bind(&visDriver::text_callback, this, _1, 0x700));
    ars408_info_subs[0x600] = node_handle.subscribe<std_msgs::String>("/info_clu_sta", 1, boost::bind(&visDriver::text_callback, this, _1, 0x600));
    ars408_info_subs[0x60A] = node_handle.subscribe<std_msgs::String>("/info_obj_sta", 1, boost::bind(&visDriver::text_callback, this, _1, 0x60A));

    motion_info_subs[0x300] = node_handle.subscribe<ars408_msg::GPSinfo>("/GPSinfo", 1, boost::bind(&visDriver::text_callback_float, this, _1, 0x300));

    overlayText_pubs[0x201] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText201", 1);
    overlayText_pubs[0x700] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText700", 1);
    overlayText_pubs[0x600] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText600", 1);
    overlayText_pubs[0x60A] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText60A", 1);
    overlayText_pubs[0x300] = node_handle.advertise<jsk_rviz_plugins::OverlayText>("/overlayText300", 1);

    filter_service = node_handle.advertiseService("/filter", &visDriver::set_filter, this);
}

void visDriver::text_callback(const std_msgs::String::ConstPtr& msg, int id)
{
    jsk_rviz_plugins::OverlayText overlaytext;
    overlaytext.action = jsk_rviz_plugins::OverlayText::ADD;
    overlaytext.text = msg->data;
    overlayText_pubs[id].publish(overlaytext);
}

void visDriver::text_callback_float(const ars408_msg::GPSinfo::ConstPtr& msg, int id)
{
    jsk_rviz_plugins::OverlayText overlaytext;
    overlaytext.action = jsk_rviz_plugins::OverlayText::ADD;
    std::stringstream ss;
    ss << "Speed" << ": " << msg->speed << std::endl;
    ss << "ZAxis" << ": " << msg->zaxis << std::endl;
    ss << std::setprecision(15) << "Longitude" << ": " << msg->longitude << std::endl;
    ss << std::setprecision(15) << "Latitude" << ": " << msg->latitude << std::endl;
    ss << "AccX" << ": " << msg->accX << std::endl;
    ss << "AccY" << ": " << msg->accY << std::endl;
    ss << "AccZ" << ": " << msg->accZ << std::endl;
    overlaytext.text = ss.str();
    overlayText_pubs[id].publish(overlaytext);

    ars408_msg::pathPoint gpsPoint;
    gpsPoint.X = msg->latitude / 0.00000899823754;
    gpsPoint.Y =((msg->longitude - 121) * cos(msg->latitude * M_PI / 180))/0.000008983152841195214 + 250000;

    if(gpsPoints.pathPoints.size() == 0){
        gpsPoints.pathPoints.push_back(gpsPoint);
    }
    else if(gpsPoint.X != gpsPoints.pathPoints[gpsPoints.pathPoints.size()-1].X || gpsPoint.Y != gpsPoints.pathPoints[gpsPoints.pathPoints.size()-1].Y){
        gpsPoints.pathPoints.push_back(gpsPoint);
    }

    #ifdef RVIZ_TRAJECTORY
    visualization_msgs::Marker world_trajectory;
    visualization_msgs::Marker trajectory;

    world_trajectory.header.frame_id = "/my_frame";
    world_trajectory.header.stamp = ros::Time::now();

    world_trajectory.ns = "world_trajectory_lines";
    world_trajectory.id = 1;
    world_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    world_trajectory.action = visualization_msgs::Marker::ADD;
    world_trajectory.pose.orientation.w = 1.0;

    world_trajectory.scale.x = 0.2;
    world_trajectory.color.r = 1.0;
    world_trajectory.color.g = 1.0;
    world_trajectory.color.a = 1.0;

    trajectory.header.frame_id = "/my_frame";
    trajectory.header.stamp = ros::Time::now();

    trajectory.ns = "trajectory_lines";
    trajectory.id = 0;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory.action = visualization_msgs::Marker::ADD;
    trajectory.pose.orientation.w = 1.0;

    trajectory.scale.x = 0.2;
    trajectory.color.r = 1.0;
    trajectory.color.g = 1.0;
    trajectory.color.a = 1.0;

    float p_x1 = gpsPoints.pathPoints[gpsPoints.pathPoints.size()-1].X;
    float p_y1 = gpsPoints.pathPoints[gpsPoints.pathPoints.size()-1].Y;
    float p_x0 = gpsPoints.pathPoints[gpsPoints.pathPoints.size()-2].X;
    float p_y0 = gpsPoints.pathPoints[gpsPoints.pathPoints.size()-2].Y;
    float d = pow(pow(p_x1 - p_x0, 2)+ pow(p_y1 - p_y0, 2) ,0.5);
    float d_y = abs(p_y1 - p_y0);
    float d_x = abs(p_x1 - p_x0);

    angle = acos (d_y / d) * 180.0 / M_PI;

    if(p_x0 > p_x1 && p_y0 < p_y1)
        angle = 90 + angle;
    else if(p_x0 > p_x1 && p_y0 > p_y1)
        angle = 270 - angle;
    else if(p_x0 < p_x1 && p_y0 > p_y1)
        angle = 270 + angle;
    else if(p_x0 < p_x1 && p_y0 < p_y1)
        angle = 90 - angle;

    if(d_y == 0 && p_x1 > p_x0)
        angle = 0;
    else if(d_y == 0 && p_x1 < p_x0)
        angle = 180;
    else if(d_x == 0 && p_y1 > p_y0)
        angle = 90;
    else if(d_x == 0 && p_y1 < p_y0)
        angle = 270;

    angle = angle * M_PI / 180;

    for (auto it = gpsPoints.pathPoints.begin(); it < gpsPoints.pathPoints.end(); ++it)
    {
        // world_trajectory
        geometry_msgs::Point world_p;
        if(init_lat == -1)
            init_lat = gpsPoint.X;

        if(init_long == -1)
            init_long = gpsPoint.Y;

        float w_longitude = -(it->Y - init_long);
        float w_latitude = (it->X - init_lat);

        world_p.z = 1;
        world_p.x = w_latitude;
        world_p.y = w_longitude;
        world_trajectory.points.push_back(world_p);

        // trajectory
        geometry_msgs::Point p;
        float longitude = -(it->Y - p_y1);
        float latitude = (it->X - p_x1);

        p.z = 1;
        p.x = cos(angle) * latitude - sin(angle) * longitude;
        p.y = sin(angle) * latitude + cos(angle) * longitude;
        trajectory.points.push_back(p);
    }
    world_trajectory_pub.publish(world_trajectory);
    trajectory_pub.publish(trajectory);
    #endif

    nowSpeed = (int)(msg->speed / 2.5) * 2.5;
    predict_speed = msg->speed * 4;
    predict_speed /= 50;

    predict_zaxis=msg->zaxis * 4;
    predict_zaxis /= 50;

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

        x1 = cos((90-predict_zaxis * i) * M_PI / 180) * predict_speed + x0;
        y1 = sin((90-predict_zaxis * i) * M_PI / 180) * predict_speed + y0;

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

    #ifdef RVIZ_RANGE
    visualization_msgs::MarkerArray range_markers;
    visualization_msgs::Marker range_marker_S;

    range_marker_S.header.frame_id = "/my_frame";
    range_marker_S.header.stamp = ros::Time::now();

    range_marker_S.ns = "range_marker_S";
    range_marker_S.id = 0;
    range_marker_S.type = visualization_msgs::Marker::LINE_STRIP;
    range_marker_S.action = visualization_msgs::Marker::ADD;
    range_marker_S.pose.orientation.w = 1.0;

    range_marker_S.scale.x = 0.5;
    range_marker_S.color.b = 1.0;
    range_marker_S.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = 1;
    float rotate;
    rotate = -40 * M_PI / 180;
    p.x = cos(rotate) * 70 - sin(rotate) * 0;
    p.y = sin(rotate) * 70 + cos(rotate) * 0;
    range_marker_S.points.push_back(p);
    rotate = -46 * M_PI / 180;
    p.x = cos(rotate) * 35 - sin(rotate) * 0;
    p.y = sin(rotate) * 35 + cos(rotate) * 0;
    range_marker_S.points.push_back(p);
    p.x = 0;
    p.y = 0;
    range_marker_S.points.push_back(p);
    rotate = 46 * M_PI / 180;
    p.x = cos(rotate) * 35 - sin(rotate) * 0;
    p.y = sin(rotate) * 35 + cos(rotate) * 0;
    range_marker_S.points.push_back(p);
    rotate = 40 * M_PI / 180;
    p.x = cos(rotate) * 70 - sin(rotate) * 0;
    p.y = sin(rotate) * 70 + cos(rotate) * 0;
    range_marker_S.points.push_back(p);
    for(int i = 40; i >= -40; i-=5){
        rotate = i * M_PI / 180;
        p.x = cos(rotate) * 70 - sin(rotate) * 0;
        p.y = sin(rotate) * 70 + cos(rotate) * 0;
        range_marker_S.points.push_back(p);
    }
    range_markers.markers.push_back(range_marker_S);

    visualization_msgs::Marker range_marker_F;

    range_marker_F.header.frame_id = "/my_frame";
    range_marker_F.header.stamp = ros::Time::now();

    range_marker_F.ns = "range_marker_F";
    range_marker_F.id = 1;
    range_marker_F.type = visualization_msgs::Marker::LINE_STRIP;
    range_marker_F.action = visualization_msgs::Marker::ADD;
    range_marker_F.pose.orientation.w = 1.0;

    range_marker_F.scale.x = 0.8;
    range_marker_F.color.r = 1.0;
    range_marker_F.color.a = 1.0;

    rotate = 4 * M_PI / 180;
    p.x = cos(rotate) * 250 - sin(rotate) * 0;
    p.y = sin(rotate) * 250 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    rotate = 9 * M_PI / 180;
    p.x = cos(rotate) * 150 - sin(rotate) * 0;
    p.y = sin(rotate) * 150 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    p.z = 1;
    p.x = 0;
    p.y = 0;
    range_marker_F.points.push_back(p);
    rotate = -9 * M_PI / 180;
    p.x = cos(rotate) * 150 - sin(rotate) * 0;
    p.y = sin(rotate) * 150 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    rotate = -4 * M_PI / 180;
    p.x = cos(rotate) * 250 - sin(rotate) * 0;
    p.y = sin(rotate) * 250 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    rotate = 4 * M_PI / 180;
    p.x = cos(rotate) * 250 - sin(rotate) * 0;
    p.y = sin(rotate) * 250 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    range_markers.markers.push_back(range_marker_F);
    range_pub.publish(range_markers);
    #endif

    visualization_msgs::MarkerArray marArr;
    visualization_msgs::MarkerArray radarPoints_marker;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Clear
    marker.action = visualization_msgs::Marker::DELETEALL;
    marArr.markers.push_back(marker);
    markerArr_pub.publish(marArr);

    ars408_msg::pathPoints pathPs;

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

        #ifdef RVIZ_RADARPOINTS_TRAJECTORY
        visualization_msgs::Marker radarPoint_marker;

        radarPoint_marker.header.frame_id = "/my_frame";
        radarPoint_marker.header.stamp = ros::Time::now();

        radarPoint_marker.ns = "radarPoint_marker";
        radarPoint_marker.id = it->id;
        radarPoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
        radarPoint_marker.action = visualization_msgs::Marker::ADD;
        radarPoint_marker.pose.orientation.w = 1.0;
        radarPoint_marker.lifetime = ros::Duration(0.1);

        radarPoint_marker.scale.x = 0.2;
        radarPoint_marker.color.r = 1.0;
        radarPoint_marker.color.g = 1.0;
        radarPoint_marker.color.b = 1.0;
        radarPoint_marker.color.a = 1.0;

        ars408_msg::pathPoint radarpoint;
        radarpoint.X = it->distX;
        radarpoint.Y = it->distY;

        int id_name = it->id;
        if(radarPoints[id_name].pathPoints.size() == 0){
            radarPoints[id_name].pathPoints.push_back(radarpoint);
        }
        else if(radarPoints[id_name].pathPoints[radarPoints[id_name].pathPoints.size()-1].X != it->distX || radarPoints[id_name].pathPoints[radarPoints[id_name].pathPoints.size()-1].Y != it->distY){
            if(pow(pow(radarpoint.X - radarPoints[id_name].pathPoints.back().X, 2)+ pow(radarpoint.Y - radarPoints[id_name].pathPoints.back().Y, 2) ,0.5) > 5){
                radarPoints[id_name].pathPoints.clear();
            }
            if(radarPoints[id_name].pathPoints.size() == 10)
                radarPoints[id_name].pathPoints.erase (radarPoints[id_name].pathPoints.begin());
            radarPoints[id_name].pathPoints.push_back(radarpoint);
        }

        for (auto it = radarPoints[id_name].pathPoints.begin(); it < radarPoints[id_name].pathPoints.end(); ++it){
            geometry_msgs::Point p;
            p.z = 1;
            p.x = it->X;
            p.y = it->Y;
            radarPoint_marker.points.push_back(p);
        }
        if(radarPoints[id_name].pathPoints.size() >= 2){
            radarPoints_marker.markers.push_back(radarPoint_marker);
        }
        #endif

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
        ss << "ID: " << it->id << std::endl;
        ss << "DynProp: " << ARS408::DynProp[it->dynProp] << std::endl;
        ss << "RCS: " << it->rcs << std::endl;
        ss << "VrelLong: " << it->vrelX << std::endl;
        ss << "VrelLat: " << it->vrelY << std::endl;
        ss << "Distance: " << sqrt(pow(it->distX, 2) + pow(it->distY, 2)) << std::endl;
        ss << "Angle: " << atan2(it->distY, it->distX) * 180 / M_PI << std::endl;

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
        #ifdef RVIZ_RADARPOINTS_TRAJECTORY
        radar_trajectory_pub.publish(radarPoints_marker);
        #endif
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
