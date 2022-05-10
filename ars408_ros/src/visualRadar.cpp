#include <string>
#include <map>
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
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
#include "eigen3/Eigen/Dense"
#include <vector>

// #define RVIZ_ARROW
// #define RVIZ_TEXT
#define RVIZ_TRAJECTORY
#define RVIZ_RANGE
#define RVIZ_RADARPOINTS_TRAJECTORY
#define RADAR_PREDICT       // AEB
#define PREDICT_TRAJECTORY
#define COLLISION_RANGE

float nowSpeed = 0;
float nowZaxis = 0;

float RCS_filter = -10000;
float predict_speed = 0;
float predict_zaxis = 0;

float init_long = -1;
float init_lat = -1;

float angle;

// Param For Three Radar
float radar2_ytrans = -0.8;
float radar3_ytrans = 0.8;
float radar2_rad = -1.22;
float radar3_rad = 1.22;
float trans;
float rad;


ars408_msg::pathPoints gpsPoints;
ars408_msg::pathPoints predict_points;


ros::Time radar_period;
ros::Time gps_period;

float radar_abs_speed[100] = {0};

class visDriver
{
    public:
        visDriver(std::string);

    private:
        std::string radarChannelframe;
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
        ros::Publisher radar_predict_pub;
        ros::Publisher predict_trajectory_pub;
        ros::Publisher collision_range_pub;

        std::map<int, ros::Subscriber> ars408_info_subs;
        std::map<int, ros::Subscriber> motion_info_subs;
        std::map<int, ros::Publisher> overlayText_pubs;
        ros::ServiceServer filter_service;

        /* Callback Functions */
        void ars408rviz_callback(const ars408_msg::RadarPoints::ConstPtr& msg);

        void text_callback(const std_msgs::String::ConstPtr& msg, int id);

        void text_callback_float(const ars408_msg::GPSinfo::ConstPtr& msg, int id);


        // TODO: GPS trajectory
        void text_callback_float(const ars408_msg::GPSinfo::ConstPtr& msg, int id);

        void rviz_range();
        visualization_msgs::Marker rviz_radar(visualization_msgs::MarkerArray& radarTraj_markers, auto it);
};

visDriver::visDriver(std::string radarChannel)
{
    radarChannelframe = radarChannel;
    world_trajectory_pub = node_handle.advertise<visualization_msgs::Marker>(radarChannel + "/world_trajectory", 1);
    trajectory_pub = node_handle.advertise<visualization_msgs::Marker>(radarChannel + "/trajectory", 1);
    radar_trajectory_pub = node_handle.advertise<visualization_msgs::MarkerArray>(radarChannel + "/radar_trajectory", 1);
    range_pub = node_handle.advertise<visualization_msgs::MarkerArray>(radarChannel + "/range", 1);
    radar_predict_pub = node_handle.advertise<visualization_msgs::MarkerArray>(radarChannel + "/radar_predict", 1);
    predict_trajectory_pub = node_handle.advertise<nav_msgs::Path>(radarChannel + "/predict_trajectory", 1);
    collision_range_pub = node_handle.advertise<visualization_msgs::MarkerArray>(radarChannel + "/collision_range", 1);

    ars408_info_subs[0x201] = node_handle.subscribe<std_msgs::String>(radarChannel + "/info_201", 1, boost::bind(&visDriver::text_callback, this, _1, 0x201));
    ars408_info_subs[0x700] = node_handle.subscribe<std_msgs::String>(radarChannel + "/info_700", 1, boost::bind(&visDriver::text_callback, this, _1, 0x700));
    ars408_info_subs[0x600] = node_handle.subscribe<std_msgs::String>(radarChannel + "/info_clu_sta", 1, boost::bind(&visDriver::text_callback, this, _1, 0x600));
    ars408_info_subs[0x60A] = node_handle.subscribe<std_msgs::String>(radarChannel + "/info_obj_sta", 1, boost::bind(&visDriver::text_callback, this, _1, 0x60A));

    motion_info_subs[0x300] = node_handle.subscribe<ars408_msg::GPSinfo>("/GPSinfo", 1, boost::bind(&visDriver::text_callback_float, this, _1, 0x300));

    overlayText_pubs[0x201] = node_handle.advertise<jsk_rviz_plugins::OverlayText>(radarChannel + "/overlayText201", 1);
    overlayText_pubs[0x700] = node_handle.advertise<jsk_rviz_plugins::OverlayText>(radarChannel + "/overlayText700", 1);
    overlayText_pubs[0x600] = node_handle.advertise<jsk_rviz_plugins::OverlayText>(radarChannel + "/overlayText600", 1);
    overlayText_pubs[0x60A] = node_handle.advertise<jsk_rviz_plugins::OverlayText>(radarChannel + "/overlayText60A", 1);

    filter_service = node_handle.advertiseService("/filter", &visDriver::set_filter, this);
}

void visDriver::rviz_range()
{
    /* Draw Radar Range on RVIZ */

    visualization_msgs::MarkerArray range_markers;
    visualization_msgs::Marker range_marker_S;
    float rotate;
    p.x = 0;
    p.y = sin(rotate) * 70 + cos(rotate) * 0;
    range_marker_S.points.push_back(p);
    for(int i = 40; i >= -40; i-=5){
        rotate = i * M_PI / 180 + rad;
        p.x = cos(rotate) * 70 - sin(rotate) * 0;
        p.y = sin(rotate) * 70 + cos(rotate) * 0;
        range_marker_S.points.push_back(p);
    }
    range_markers.markers.push_back(range_marker_S);

    // Red Range (Narrow One)
    visualization_msgs::Marker range_marker_F;

    range_marker_F.header.frame_id = radarChannelframe;
    range_marker_F.header.stamp = ros::Time::now();

    range_marker_F.ns = "range_marker_F";
    range_marker_F.id = 1;
    range_marker_F.type = visualization_msgs::Marker::LINE_STRIP;
    range_marker_F.action = visualization_msgs::Marker::ADD;
    range_marker_F.pose.orientation.w = 1.0;

    range_marker_F.scale.x = 0.8;
    range_marker_F.color.r = 1.0;
    range_marker_F.color.a = 1.0;

    rotate = 4 * M_PI / 180 + rad;
    p.x = cos(rotate) * 250 - sin(rotate) * 0;
    p.y = sin(rotate) * 250 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    rotate = 9 * M_PI / 180 + rad;
    p.x = cos(rotate) * 150 - sin(rotate) * 0;
    p.y = sin(rotate) * 150 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    p.z = 1;
    p.x = 0;
    p.y = 0 + trans;
    range_marker_F.points.push_back(p);
    rotate = -9 * M_PI / 180 + rad;
    p.x = cos(rotate) * 150 - sin(rotate) * 0;
    p.y = sin(rotate) * 150 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    rotate = -4 * M_PI / 180 + rad;
    p.x = cos(rotate) * 250 - sin(rotate) * 0;
    p.y = sin(rotate) * 250 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    rotate = 4 * M_PI / 180 + rad;
    p.x = cos(rotate) * 250 - sin(rotate) * 0;
    p.y = sin(rotate) * 250 + cos(rotate) * 0;
    range_marker_F.points.push_back(p);
    range_markers.markers.push_back(range_marker_F);
    range_pub.publish(range_markers);
}

visualization_msgs::Marker visDriver::rviz_radar(visualization_msgs::MarkerArray& radarTraj_markers, auto it)
{
    /* 
    * Draw Radar Points and Radar Trajectory on RVIZ
    */
    visualization_msgs::Marker marker_sphere;
    marker_sphere.header.frame_id = radarChannelframe;
    marker_sphere.header.stamp = ros::Time::now();

    marker_sphere.ns = "sphere";
    marker_sphere.id = it->id;
    marker_sphere.type = visualization_msgs::Marker::SPHERE;
    marker_sphere.action = visualization_msgs::Marker::ADD;
    marker_sphere.pose.position.x = it->distX;
    marker_sphere.pose.position.y = it->distY;
    marker_sphere.pose.position.z = 0.05;
    marker_sphere.scale.x = 1;
    marker_sphere.scale.y = 1;
    marker_sphere.scale.z = 0.1;

    marker_sphere.pose.orientation.z = 1.0;
    marker_sphere.pose.orientation.x = 0.0;
    marker_sphere.pose.orientation.y = 0.0;

    #ifdef RVIZ_RADARPOINTS_TRAJECTORY
    /*
    * Past trajectory of radar points.
    */
    visualization_msgs::Marker radarTraj_marker;

    radarTraj_marker.header.frame_id = radarChannelframe;
    radarTraj_marker.header.stamp = ros::Time::now();
        
    radarTraj_marker.ns = "radarTraj_marker";
    radarTraj_marker.id = it->id;
    radarTraj_marker.type = visualization_msgs::Marker::LINE_STRIP;
    radarTraj_marker.action = visualization_msgs::Marker::ADD;
    radarTraj_marker.pose.orientation.w = 1.0;
    radarTraj_marker.lifetime = ros::Duration(0.1);

    radarTraj_marker.scale.x = 0.5;
    radarTraj_marker.color.r = 1.0;
    radarTraj_marker.color.g = 1.0;
    radarTraj_marker.color.b = 1.0;
    radarTraj_marker.color.a = 1.0;

    ars408_msg::pathPoint radarpoint;
    radarpoint.X = it->distX;
    radarpoint.Y = it->distY;
    int id_name = it->id;

    if(radarTraj[id_name].pathPoints.size() == 0){
        radarTraj[id_name].pathPoints.push_back(radarpoint);
    }
    else if(radarTraj[id_name].pathPoints[radarTraj[id_name].pathPoints.size()-1].X != it->distX || radarTraj[id_name].pathPoints[radarTraj[id_name].pathPoints.size()-1].Y != it->distY){
        if(pow(pow(radarpoint.X - radarTraj[id_name].pathPoints.back().X, 2)+ pow(radarpoint.Y - radarTraj[id_name].pathPoints.back().Y, 2) ,0.5) > 3){
            radarTraj[id_name].pathPoints.clear();
            radar_abs_speed[id_name] = 0;
        }
        if(radarTraj[id_name].pathPoints.size() == 5)
            radarTraj[id_name].pathPoints.erase (radarTraj[id_name].pathPoints.begin());
        radarTraj[id_name].pathPoints.push_back(radarpoint);
    }

    // ROS point to RVIZ geometry
    for (auto it = radarTraj[id_name].pathPoints.begin(); it < radarTraj[id_name].pathPoints.end(); ++it){
        geometry_msgs::Point p;
        p.z = 1;
        p.x = it->X;
        p.y = it->Y;
        radarTraj_marker.points.push_back(p);
    }
    if(radarTraj[id_name].pathPoints.size() >= 2){
        radarTraj_markers.markers.push_back(radarTraj_marker);
    }
    #endif

    return marker_sphere;
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
    if (radarChannelframe != "/radar/first")
        return;
    
    float time_diff = (ros::Time::now() - gps_period).toSec();
    gps_period = ros::Time::now();

    ars408_msg::pathPoint gpsPoint;
    gpsPoint.X = msg->latitude / 0.00000899823754;
    gpsPoint.Y =((msg->longitude - 121) * cos(msg->latitude * M_PI / 180))/0.000008983152841195214 + 250000;

     bool is_gps = false;

    if(gpsPoints.pathPoints.size() == 0 || gpsPoint.X != gpsPoints.pathPoints[gpsPoints.pathPoints.size()-1].X || gpsPoint.Y != gpsPoints.pathPoints[gpsPoints.pathPoints.size()-1].Y){
        is_gps = true;
        gpsPoints.pathPoints.push_back(gpsPoint);
        gps_timeDiff.push_back(time_diff);
    }

    #ifdef RVIZ_TRAJECTORY
    visualization_msgs::Marker world_trajectory;
    visualization_msgs::Marker trajectory;

    world_trajectory.header.frame_id = radarChannelframe;
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

    trajectory.header.frame_id = radarChannelframe;
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

    float i = 0;

    float pre_long = 0, pre_lat = 0;

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
        i++;
    }

    world_trajectory_pub.publish(world_trajectory);
    trajectory_pub.publish(trajectory);
    #endif

    #ifdef PREDICT_TRAJECTORY
    if(is_gps){
        nav_msgs::Path predict_trajectory;
        predict_trajectory.header.frame_id = radarChannelframe;
        predict_trajectory.header.stamp = ros::Time::now();

        ars408_msg::pathPoint predict_point;
        predict_point.X = (sin((90 - msg->zaxis) * M_PI / 180) * (msg->speed));
        predict_point.Y = (cos((90 - msg->zaxis) * M_PI / 180) * (msg->speed));
        predict_points.pathPoints.push_back(predict_point);

        angle = -msg->zaxis + 180;
        angle = angle * M_PI / 180;

        if(predict_points.pathPoints.size() > 0)
            for(int i = 0; i < predict_points.pathPoints.size(); i++){
                predict_points.pathPoints[i].X += predict_point.X;
                predict_points.pathPoints[i].Y += predict_point.Y;

                if(i != predict_points.pathPoints.size() -1){
                    predict_points.pathPoints[i].X = cos(msg->zaxis * M_PI / 180) * predict_points.pathPoints[i].X - sin(msg->zaxis* M_PI / 180) * predict_points.pathPoints[i].Y;
                    predict_points.pathPoints[i].Y = sin(msg->zaxis * M_PI / 180) * predict_points.pathPoints[i].X + cos(msg->zaxis* M_PI / 180) * predict_points.pathPoints[i].Y;
                }

                geometry_msgs::PoseStamped ps;
                geometry_msgs::Point p;
                
                p.z = -1;
                p.x = cos(angle) * predict_points.pathPoints[i].X - sin(angle) * predict_points.pathPoints[i].Y;
                p.y = -(sin(angle) * predict_points.pathPoints[i].X + cos(angle) * predict_points.pathPoints[i].Y);

                ps.pose.position = p;
                predict_trajectory.poses.push_back(ps);

                if(i == predict_points.pathPoints.size() - 1){
                    p.x = 0;
                    p.y = 0;
                    ps.pose.position = p;
                    predict_trajectory.poses.push_back(ps);
                }
            }
        predict_trajectory_pub.publish(predict_trajectory);
    }
    #endif
    
    nowSpeed = msg->speed;
    nowZaxis = msg->zaxis;
}

void visDriver::ars408rviz_callback(const ars408_msg::RadarPoints::ConstPtr& msg)
{
    float time_diff = (ros::Time::now() - radar_period).toSec();
    radar_period = ros::Time::now();

    #ifdef RVIZ_RANGE
    /*
    * RVIZ RADAR RANGE
    */
    rviz_range();
    #endif

    /*
    * [1] Show Radar
    * [2] Kalman predict Radar Path
    * [3] AEB
    */
    visualization_msgs::MarkerArray collision_markers;
    visualization_msgs::MarkerArray marArr;
    visualization_msgs::MarkerArray radarTraj_markers;
    visualization_msgs::MarkerArray kalman_markers;
    std_msgs::Int8MultiArray colli_arr;

    visualization_msgs::Marker marker;
    marker.header.frame_id = radarChannelframe;
    marker.header.stamp = ros::Time::now();

    // Clear all the markers
    marker.action = visualization_msgs::Marker::DELETEALL;
    marArr.markers.push_back(marker);
    markerArr_pub.publish(marArr);

    ars408_msg::pathPoints pathPs;

    int last_id = -1;
    std_msgs::Int8MultiArray aeb_arr;

    for (auto it = msg->rps.begin(); it != msg->rps.end(); ++it)
    {   
        /*
        * Radar Speed calculation
        */
        float pov_speed = sqrt(pow(it->vrelX, 2) + pow(it->vrelY, 2));

        if(it->vrelX < 0)
            pov_speed = -pov_speed;

        float acc = pov_speed - radar_abs_speed[it->id];

        radar_abs_speed[it->id] = pov_speed;

        int id_name = it->id;

        /*
        * RVIZ RADAR SHOW
        */
        visualization_msgs::Marker marker_sphere = rviz_radar(radarTraj_markers, it);
        if (it->classT == 0x00)
        {
            // White: point
            marker_sphere.ns = "point";
            marker_sphere.color.r = 1.0f;
            marker_sphere.color.g = 1.0f;
            marker_sphere.color.b = 1.0f;
            marker_sphere.color.a = 1.0f;
        }
        else if (it->classT == 0x01)
        {
            // Red: car
            marker_sphere.ns = "car";
            marker_sphere.color.r = 1.0f;
            marker_sphere.color.g = 0.0f;
            marker_sphere.color.b = 0.0f;
            marker_sphere.color.a = 1.0f;
        }
        else if (it->classT == 0x02)
        {
            // Purple： truck
            marker_sphere.ns = "truck";
            marker_sphere.color.r = 1.0f;
            marker_sphere.color.g = 0.0f;
            marker_sphere.color.b = 1.0f;
            marker_sphere.color.a = 1.0f;
        }
        else if (it->classT == 0x03 || it->classT==0x07)
        {
            // Blue: reserved
            marker_sphere.ns = "reserved";
            marker_sphere.color.r = 0.0f;
            marker_sphere.color.g = 0.0f;
            marker_sphere.color.b = 1.0f;
            marker_sphere.color.a = 1.0f;
        }
        else if (it->classT == 0x04)
        {
            // Yellow: motorcycle
            marker_sphere.ns = "motorcycle";
            marker_sphere.color.r = 1.0f;
            marker_sphere.color.g = 1.0f;
            marker_sphere.color.b = 0.0f;
            marker_sphere.color.a = 1.0;
        }
        else if (it->classT == 0x05)
        {
            // Green: bicycle
            marker_sphere.ns = "bicycle";
            marker_sphere.color.r = 0.0f;
            marker_sphere.color.g = 1.0f;
            marker_sphere.color.b = 0.0f;
            marker_sphere.color.a = 1.0f;
        }
        else if (it->classT == 0x06)
        {
            // Cyan: wide
            marker_sphere.ns = "wide";
            marker_sphere.color.r = 0.0f;
            marker_sphere.color.g = 1.0f;
            marker_sphere.color.b = 1.0f;
            marker_sphere.color.a = 1.0f;
        }
        else
        {
            // Orange: others
            marker_sphere.ns = "others";
            marker_sphere.color.r = 1.0f;
            marker_sphere.color.g = 0.5f;
            marker_sphere.color.b = 0.0f;
            marker_sphere.color.a = 1.0f;
        }

        if (it->isDanger)
        {
            // gray: Danger
            marker_sphere.ns = "Danger";
            marker_sphere.color.r = 0.7f;
            marker_sphere.color.g = 0.7f;
            marker_sphere.color.b = 0.7f;
            marker_sphere.color.a = 1.0f;
        }

        
        #ifdef RVIZ_TEXT
        // Text
        visualization_msgs::Marker marker_text;
        marker_text.header.frame_id = radarChannelframe;
        marker_text.header.stamp = ros::Time::now();

        marker_text.ns = "text";
        marker_text.id = it->id;

        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::Marker::ADD;

        float vrel = pow(pow(it->vrelX, 2)+pow(it->vrelY, 2), 0.5);
        if(it->vrelX < 0)
            vrel = -vrel;

        std::stringstream ss;
        ss << "ID: " << it->id << std::endl;
        // ss << "DynProp: " << ARS408::DynProp[it->dynProp] << std::endl;
        // ss << "RCS: " << it->rcs << std::endl;
        ss << "DistX: " << it->distX << std::endl;
        ss << "DistY: " << it->distY << std::endl;
        ss << "VrelLong: " << it->vrelX << std::endl;
        ss << "VrelLat: " << it->vrelY << std::endl;
        ss << "Distance: " << sqrt(pow(it->distX, 2) + pow(it->distY, 2)) << std::endl;
        // ss << "Abs_speed: " << radar_abs_speed[it->id] << std::endl;
        ss << "Vrel: " << vrel << std::endl;
        ss << "car_speed: " << nowSpeed << std::endl;
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
            marker_arrow.header.frame_id = radarChannelframe;
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
            marArr.markers.push_back(marker_sphere);
            #ifdef RVIZ_TEXT
            marArr.markers.push_back(marker_text);
            #endif
        }
    }

    if (msg->rps.size() > 0) {
        radar_predict_pub.publish(kalman_markers);
        markerArr_pub.publish(marArr);
        #ifdef RVIZ_RADARPOINTS_TRAJECTORY
        radar_trajectory_pub.publish(radarTraj_markers);
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
    std::string radarChannel;
    if (ros::param::get("visualRadar/radarChannel", radarChannel))
    {
        std::cout << "Get parm:" << radarChannel << std::endl;
    }
    else
    {
        std::cout << "Get Name Error" << std::endl;
    }

    if (radarChannel != "/radar/first") {
        #undef RADAR_PREDICT
        #undef COLLISION_RANGE
    }

    visDriver node(radarChannel);
    ros::Rate r(60);

    for(int i = 0; i < 100; i++){
        disappear_time[i] = 0;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}