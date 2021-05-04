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
// #define RVIZ_TRAJECTORY
#define RVIZ_RANGE
#define RVIZ_RADARPOINTS_TRAJECTORY
#define RADAR_PREDICT
#define PREDICT_TRAJECTORY
#define COLLISION_RANGE

float nowSpeed = 0;
float nowZaxis = 0;
float car_width = 1;
float car_length = 5;
float radar_width = 0.5;

float RCS_filter = -10000;
float predict_speed = 0;
float predict_zaxis = 0;

float init_long = -1;
float init_lat = -1;
ars408_msg::pathPoints gpsPoints;

float angle;

std::map<int, ars408_msg::pathPoints> radarPoints;

std::map<int, int> disappear_time;

ars408_msg::pathPoints collision_path;

ros::Time radar_period;

ros::Time gps_period;

Eigen::MatrixXd Q_radar(4,4);
Eigen::MatrixXd R_radar(4,4);
Eigen::MatrixXd H_radar(4,4);
Eigen::MatrixXd B_radar(4,1);
Eigen::MatrixXd U_radar(1,1);

std::vector<float> gps_timeDiff;

ars408_msg::pathPoints predict_points;

float radar_abs_speed[100] = {0};

void kf_predict(Eigen::MatrixXd& X, Eigen::MatrixXd& P, Eigen::MatrixXd F, Eigen::MatrixXd B, Eigen::MatrixXd U, Eigen::MatrixXd Q){
    X = F * X ;
    P = F * P * F.transpose() + Q;
}

void kf_update(Eigen::MatrixXd& X, Eigen::MatrixXd& P, Eigen::MatrixXd Y, Eigen::MatrixXd H, Eigen::MatrixXd R){
    Eigen::MatrixXd V =  Y - H * X;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    X = X + K * V;
    P = P - K * H * P;
}

bool intersect(std::vector<Eigen::Vector2f> a, std::vector<Eigen::Vector2f> b){
    for(int i = 0;i < a.size(); i++){
        Eigen::Vector2f cur = a[i];
        Eigen::Vector2f next = a[(i + 1) % a.size()];
        Eigen::Vector2f edge = next - cur; 

        Eigen::Vector2f axis(-edge(1), edge(0));

        float aMax = -std::numeric_limits<float>::infinity();
        float aMin = std::numeric_limits<float>::infinity();
        float bMax = -std::numeric_limits<float>::infinity();
        float bMin = std::numeric_limits<float>::infinity();

        for(const auto& v : a){
            float proj =  axis.dot(v);
            if(proj < aMin)  aMin = proj;
            if(proj > aMax)  aMax = proj;
        }

        for(const auto& v : b){
            float proj =  axis.dot(v);
            if(proj < bMin)  bMin = proj;
            if(proj > bMax)  bMax = proj;
        }

        if((aMin < bMax && aMax > bMin) || (bMin < aMax && bMin > aMin))
            continue;
        else
            return false;
    }
    return true;
}

Eigen::Vector2f rotate_point(float angle, float x, float y, float center_x, float center_y){
    float rotate = angle * M_PI / 180;

    x -= center_x;
    y -= center_y;

    float rotate_x = cos(rotate) * x - sin(rotate) * y + center_x;
    float rotate_y = sin(rotate) * x + cos(rotate) * y + center_y;

    rotate_x = abs(rotate_x) < 0.001 ? 0 : rotate_x;
    rotate_y = abs(rotate_y) < 0.001 ? 0 : rotate_y;

    return Eigen::Vector2f(rotate_x, rotate_y);
}

float min_max(float value1, float value2, int flag){
    // min
    if(flag == 0){    
        if(value1 > value2)
            return value2;
        else
            return value1;
    }
    // max
    else{    
        if(value1 > value2)
            return value1;
        else
            return value2;
    }
}

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
        ros::Publisher radar_predict_pub;
        ros::Publisher collision_pub;
        ros::Publisher kalman_predcit_pub;
        ros::Publisher predict_trajectory_pub;
        ros::Publisher collision_range_pub;
        ros::Publisher aeb_pub;

        std::map<int, ros::Subscriber> ars408_info_subs;
        std::map<int, ros::Subscriber> motion_info_subs;
        std::map<int, ros::Publisher> overlayText_pubs;
        ros::ServiceServer filter_service;

        void ars408rviz_callback(const ars408_msg::RadarPoints::ConstPtr& msg);
        void text_callback(const std_msgs::String::ConstPtr& msg, int id);
        void text_callback_float(const ars408_msg::GPSinfo::ConstPtr& msg, int id);
        bool set_filter(ars408_srv::Filter::Request &req, ars408_srv::Filter::Response &res);
        void gps(const std_msgs::Float32::ConstPtr& msg, std::string topicName, int id);
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
    radar_predict_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/radar_predict", 1);
    collision_pub = node_handle.advertise<std_msgs::Int8MultiArray>("/collision", 1);
    aeb_pub = node_handle.advertise<std_msgs::Int8MultiArray>("/aeb", 1);
    predict_trajectory_pub = node_handle.advertise<nav_msgs::Path>("/predict_trajectory", 1);
    collision_range_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/collision_range", 1);

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

    // motion_info_subs[0x300] = node_handle.subscribe<std_msgs::Float32>("/speed", 10, boost::bind(&visDriver::gps, this, _1, "Speed", 0x300));
    // motion_info_subs[0x301] = node_handle.subscribe<std_msgs::Float32>("/zaxis", 10, boost::bind(&visDriver::gps, this, _1, "ZAxis", 0x301));
}


// void visDriver::gps(const std_msgs::Float32::ConstPtr& msg, std::string topicName, int id)
// {
//     if (topicName == "Speed"){
//         nowSpeed = msg->data;
//         predict_speed = msg->data * 4;
//         predict_speed /= 100;
//     }
//     else{
//         predict_zaxis= msg->data * 4;
//         predict_zaxis /= 100;
//     }

//     std::cout<<"Speed: "<<nowSpeed<<std::endl;

//     visualization_msgs::Marker marker_predict;
//     nav_msgs::Path predict_path;
//     predict_path.header.frame_id = "/my_frame";
//     predict_path.header.stamp = ros::Time::now();

//     ars408_msg::pathPoints pathPs;

//     float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    
//     for(uint32_t i = 0; i <= 100; i++)
//     {
//         geometry_msgs::PoseStamped ps;
//         geometry_msgs::Point p;
        
//         p.z = -1;

//         x1 = cos((90-predict_zaxis * i) * M_PI / 180) * predict_speed + x0;
//         y1 = sin((90-predict_zaxis * i) * M_PI / 180) * predict_speed + y0;

//         if(i == 0){
//             x1 = 0;
//             y1 = 0;
//         }

//         p.x = y1;
//         p.y = x1;

//         x0 = x1;
//         y0 = y1;

//         ars408_msg::pathPoint temp;
//         temp.X = p.x;
//         temp.Y = p.y;
//         pathPs.pathPoints.push_back(temp);

//         ps.pose.position = p;
//         predict_path.poses.push_back(ps);
//     }
//     collision_path = pathPs;
//     pathPoints_pub.publish(pathPs);
//     predict_pub.publish(predict_path);
// }

void visDriver::text_callback(const std_msgs::String::ConstPtr& msg, int id)
{
    jsk_rviz_plugins::OverlayText overlaytext;
    overlaytext.action = jsk_rviz_plugins::OverlayText::ADD;
    overlaytext.text = msg->data;
    overlayText_pubs[id].publish(overlaytext);
}

void visDriver::text_callback_float(const ars408_msg::GPSinfo::ConstPtr& msg, int id)
{
    float time_diff = (ros::Time::now() - gps_period).toSec();
    gps_period = ros::Time::now();

    jsk_rviz_plugins::OverlayText overlaytext;
    overlaytext.action = jsk_rviz_plugins::OverlayText::ADD;
    std::stringstream ss;
    ss << "Speed" << ": " << msg->speed << std::endl;
    // ss << "ZAxis" << ": " << msg->zaxis << std::endl;
    // ss << std::setprecision(15) << "Longitude" << ": " << msg->longitude << std::endl;
    // ss << std::setprecision(15) << "Latitude" << ": " << msg->latitude << std::endl;
    // ss << "AccX" << ": " << msg->accX << std::endl;
    // ss << "AccY" << ": " << msg->accY << std::endl;
    // ss << "AccZ" << ": " << msg->accZ << std::endl;
    overlaytext.text = ss.str();
    overlayText_pubs[id].publish(overlaytext);

    // std::cout<<ss.str()<<std::endl;

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

    // nowSpeed = (int)(msg->speed / 2.5) * 2.5;
    nowSpeed = msg->speed;
    nowZaxis = msg->zaxis;
    predict_speed = msg->speed * 4;
    predict_speed /= 100;

    predict_zaxis= msg->zaxis * 4;
    predict_zaxis /= 100;
    

    #ifdef PREDICT_TRAJECTORY
    if(is_gps){
        nav_msgs::Path predict_trajectory;
        predict_trajectory.header.frame_id = "/my_frame";
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

                // without correction
                // p.x = predict_points.pathPoints[i].X;
                // p.y = predict_points.pathPoints[i].Y;

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
    
    visualization_msgs::Marker marker_predict;
    nav_msgs::Path predict_path;
    predict_path.header.frame_id = "/my_frame";
    predict_path.header.stamp = ros::Time::now();

    ars408_msg::pathPoints pathPs;

    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    
    for(uint32_t i = 0; i <= 100; i++)
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
    collision_path = pathPs;
    pathPoints_pub.publish(pathPs);
    predict_pub.publish(predict_path);
}

void visDriver::ars408rviz_callback(const ars408_msg::RadarPoints::ConstPtr& msg)
{
    float time_diff = (ros::Time::now() - radar_period).toSec();
    radar_period = ros::Time::now();

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

    visualization_msgs::MarkerArray collision_markers;
    visualization_msgs::MarkerArray marArr;
    visualization_msgs::MarkerArray radarPoints_marker;
    visualization_msgs::MarkerArray kalman_markers;
    std_msgs::Int8MultiArray colli_arr;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Clear
    marker.action = visualization_msgs::Marker::DELETEALL;
    marArr.markers.push_back(marker);
    markerArr_pub.publish(marArr);

    ars408_msg::pathPoints pathPs;

    int last_id = -1;
    std_msgs::Int8MultiArray aeb_arr;
    for (auto it = msg->rps.begin(); it != msg->rps.end(); ++it)
    {
        // float pov_distance = sqrt(pow(it->distX, 2) + pow(it->distY, 2));
        float pov_speed = sqrt(pow(it->vrelX, 2) + pow(it->vrelY, 2));

        // float braking_distance = pow(pov_speed, 2) / (2 * 9.8 * 0.7) + 3;

        if(it->vrelX < 0)
            pov_speed = -pov_speed;

        // pov_speed += nowSpeed;

        float acc = pov_speed - radar_abs_speed[it->id];

        // if(abs(it->distY) < 2 && pov_distance < braking_distance && it->vrelX < 0){
        //     std_msgs::Int8 aeb;
        //     aeb.data = it->id;
        //     aeb_arr.data.push_back(it->id);
        //     std::cout<<"=== BRAKINE === " << it->id << " acc: " << acc/time_diff << " braking_d: " << braking_distance <<" pov_d: "<<pov_distance<< std::endl;
        // }

        radar_abs_speed[it->id] = pov_speed;

        disappear_time[it->id] = 0;
        if(last_id + 1 != it->id)
            for(int i = last_id + 1; i < it->id; i++){
                disappear_time[i] +=1;
                // check disappear point
                // std::cout << "(" << i << ":" << disappear_time[i] << ") ";
                if(disappear_time[i] >= 3){
                    radarPoints[i].pathPoints.clear();
                }
            }
        last_id = it->id;
        // check disappear point
        // std::cout<< it->id << " ";

        float radar_speed = pow(pow(it->vrelX, 2) + pow(it->vrelY, 2), 0.5);

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

        radarPoint_marker.scale.x = 0.5;
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
            if(pow(pow(radarpoint.X - radarPoints[id_name].pathPoints.back().X, 2)+ pow(radarpoint.Y - radarPoints[id_name].pathPoints.back().Y, 2) ,0.5) > 3){
                radarPoints[id_name].pathPoints.clear();
                radar_abs_speed[id_name] = 0;
            }
            if(radarPoints[id_name].pathPoints.size() == 5)
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

        #ifdef RADAR_PREDICT
        if(radarPoints[id_name].pathPoints.size() >= 2){    
            Eigen::MatrixXd F_radar(4, 4);
            Eigen::MatrixXd P_radar(4, 4);
            Eigen::MatrixXd Y_radar(4, 1);
            Eigen::MatrixXd X_radar(4, 1);

            float init_x = radarPoints[id_name].pathPoints[0].X;
            float init_y = radarPoints[id_name].pathPoints[0].Y;
            float init_vx = (radarPoints[id_name].pathPoints[1].X - radarPoints[id_name].pathPoints[0].X)/time_diff;
            float init_vy = (radarPoints[id_name].pathPoints[1].Y - radarPoints[id_name].pathPoints[0].Y)/time_diff;

            X_radar << init_x,
                 init_y,
                 init_vx,
                 init_vy;

            F_radar << 1, 0, time_diff, 0,
                 0, 1, 0, time_diff, 
                 0, 0, 1, 0, 
                 0, 0, 0, 1;

            P_radar << 0.1, 0, 0, 0,
                 0, 0.1, 0, 0,
                 0, 0, 0.1, 0,
                 0, 0, 0, 0.1;
            
            float cur_radar_speed;

            for(int t = 1; t < radarPoints[id_name].pathPoints.size(); t+=1){
                float velocity_x = (radarPoints[id_name].pathPoints[t].X - radarPoints[id_name].pathPoints[t - 1].X)/time_diff;
                float velocity_y = (radarPoints[id_name].pathPoints[t].Y - radarPoints[id_name].pathPoints[t - 1].Y)/time_diff;

                if(t == radarPoints[id_name].pathPoints.size() - 1){
                    F_radar <<  1, 0, 4, 0,
                                0, 1, 0, 4,
                                0, 0, 1, 0,
                                0, 0, 0, 1;
                    cur_radar_speed = pow(pow(velocity_x, 2) + pow(velocity_y, 2), 0.5);
                }
                
                Y_radar << radarPoints[id_name].pathPoints[t].X, radarPoints[id_name].pathPoints[t].Y, velocity_x, velocity_y;

                kf_predict(X_radar, P_radar, F_radar, B_radar, U_radar, Q_radar);
                kf_update(X_radar, P_radar, Y_radar, H_radar, R_radar);
            }

            visualization_msgs::Marker kalman_marker;

            kalman_marker.header.frame_id = "/my_frame";
            kalman_marker.header.stamp = ros::Time::now();

            kalman_marker.ns = "kalman_markers";
            kalman_marker.id = it->id;
            kalman_marker.type = visualization_msgs::Marker::LINE_STRIP;
            kalman_marker.action = visualization_msgs::Marker::ADD;
            kalman_marker.lifetime = ros::Duration(0.1);
            
            kalman_marker.pose.orientation.w = 1.0;

            kalman_marker.scale.x = 0.5;
            kalman_marker.scale.z = 0.5;
            kalman_marker.color.r = 1.0;
            kalman_marker.color.g = 1.0;
            kalman_marker.color.b = 1.0;
            kalman_marker.color.a = 1.0;

            #ifdef COLLISION_RANGE

            std::vector<Eigen::Vector2f> a_points;
            // std::vector<Eigen::Vector2f> b_points;

            // a_points.push_back(Eigen::Vector2f(13.2899, 0.992602));
            // a_points.push_back(Eigen::Vector2f(13.3036, -0.507336));
            // a_points.push_back(Eigen::Vector2f(8.30384, -0.552949));
            // a_points.push_back(Eigen::Vector2f(8.29016, 0.946989));

            // b_points.push_back(Eigen::Vector2f(20, -0.35));
            // b_points.push_back(Eigen::Vector2f(20, -0.85));
            // b_points.push_back(Eigen::Vector2f(13.0886, -0.85));
            // b_points.push_back(Eigen::Vector2f(13.0886, -0.35));

            // std::cout<< intersect(a_points, b_points)<<"\n";
            // std::cout<< intersect(b_points, a_points) << "\n\n";

            float radar_m = (X_radar(0,0) - it->distX) / (X_radar(1,0) - it->distY);
            float radar_angle = atan(radar_m) / (M_PI / 180);
            float radar_rotate_angle = 90 - abs(radar_angle);
            if(radar_angle < 0)
                radar_rotate_angle = -radar_rotate_angle;
            
            if(pov_speed < 0){
                a_points.push_back(rotate_point(radar_rotate_angle, it->distX, it->distY + radar_width / 2, it->distX, it->distY));
                a_points.push_back(rotate_point(radar_rotate_angle, it->distX, it->distY - radar_width / 2, it->distX, it->distY));
                a_points.push_back(rotate_point(radar_rotate_angle, X_radar(0,0), X_radar(1,0) - radar_width / 2, X_radar(0,0), X_radar(1,0)));
                a_points.push_back(rotate_point(radar_rotate_angle, X_radar(0,0), X_radar(1,0) + radar_width / 2, X_radar(0,0), X_radar(1,0)));
                
                int path_index = 0;
                for (auto pred = collision_path.pathPoints.begin(); pred < collision_path.pathPoints.end(); ++pred){
                    path_index++;
                    if(path_index % 4 == 1)
                        continue;
                    std::vector<Eigen::Vector2f> b_points;

                    b_points.push_back(rotate_point(nowZaxis/100*path_index, pred->X, pred->Y + (car_width / 2), pred->X, pred->Y));
                    b_points.push_back(rotate_point(nowZaxis/100*path_index, pred->X, pred->Y - (car_width / 2), pred->X, pred->Y));
                    b_points.push_back(rotate_point(nowZaxis/100*path_index, pred->X - car_length, pred->Y - (car_width / 2), pred->X, pred->Y));
                    b_points.push_back(rotate_point(nowZaxis/100*path_index, pred->X - car_length, pred->Y + (car_width / 2), pred->X, pred->Y));

                    if(intersect(a_points, b_points) && intersect(b_points, a_points)){

                        float ttr_ego =  float(4 * path_index)/100;
                        float ttr_target = pow(pow(pred->X - it->distX, 2) + pow(pred->Y - it->distY, 2),0.5)/ abs(pov_speed);
                        float ttc_threshold = abs(pov_speed) / (2 * 9.8 * 0.5) + (abs(pov_speed) * 1.5);

                        if(abs(ttr_target - ttr_ego) < 1 && min_max(ttr_target, ttr_ego, 0) < ttc_threshold){
                            std_msgs::Int8 colli;
                            colli.data = id_name;
                            colli_arr.data.push_back(it->id);
                        }

                        ttc_threshold = (abs(pov_speed) / (2 * 9.8 * 0.3));
                        if(abs(ttr_target - ttr_ego) < 1 && min_max(ttr_target, ttr_ego, 0) < ttc_threshold){
                            std_msgs::Int8 aeb;
                            aeb.data = it->id;
                            aeb_arr.data.push_back(it->id);
                        }
                        break;
                    }
                }
            }
            
            #endif
            geometry_msgs::Point p;
            p.z = 1;
            p.x = it->distX;
            p.y = it->distY;
            kalman_marker.points.push_back(p);
            p.x = X_radar(0,0);
            p.y = X_radar(1,0);
            kalman_marker.points.push_back(p);

            if(abs(X_radar(0,0) - it->distX) < 100 && abs(X_radar(1,0) - it->distY) < 100)
                kalman_markers.markers.push_back(kalman_marker);
        }
        collision_pub.publish(colli_arr);
        aeb_pub.publish(aeb_arr);
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

        float vrel = pow(pow(it->vrelX, 2)+pow(it->vrelY, 2), 0.5);
        if(it->vrelX < 0)
            vrel = -vrel;

        std::stringstream ss;
        ss << "ID: " << it->id << std::endl;
        // ss << "DynProp: " << ARS408::DynProp[it->dynProp] << std::endl;
        // ss << "RCS: " << it->rcs << std::endl;
        // ss << "VrelLong: " << it->vrelX << std::endl;
        // ss << "VrelLat: " << it->vrelY << std::endl;
        ss << "Distance: " << sqrt(pow(it->distX, 2) + pow(it->distY, 2)) << std::endl;
        ss << "Abs_speed: " << radar_abs_speed[it->id] << std::endl;
        ss << "Vrel: " << vrel << std::endl;
        ss << "car_speed: " << nowSpeed << std::endl;
        // ss << "Angle: " << atan2(it->distY, it->distX) * 180 / M_PI << std::endl;

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

    for(int i = last_id + 1; i < 100; i++){
        disappear_time[i] +=1;
        // std::cout << "(" << i << ":" << disappear_time[i] << ") ";
        if(disappear_time[i] >= 3){
            radarPoints[i].pathPoints.clear();
        }
    }

    if (msg->rps.size() > 0) {
        radar_predict_pub.publish(kalman_markers);
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
    // std::cout<< "server response: " << res.RCS_filter << std::endl;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualRadar");
    visDriver node;
    ros::Rate r(60);

    Q_radar <<  4e-4, 0, 0, 0,
                0, 4e-4, 0, 0,
                0, 0, 4e-4, 0,
                0, 0, 0, 4e-4;

    R_radar <<  0.01, 0, 0, 0,     
                0, 0.01, 0, 0,
                0, 0, 0.01, 0,
                0, 0, 0, 0.01;

    H_radar <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0 ,1;

    B_radar << 0, 0, 0, 0;
    U_radar << 0;

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