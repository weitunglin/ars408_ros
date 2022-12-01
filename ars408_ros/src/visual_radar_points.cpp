#include <ros/ros.h>
#include <ros/console.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/ColorRGBA.h>
#include <ars408_ros/ARS408_CAN.h>
#include <ars408_msg/RadarPoints.h>
#include <string>
#include <map>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <sstream>

class Radar_points_visualizer
{
    public:
        Radar_points_visualizer();
    private:
        ros::Subscriber transformed_radar_sub;
        ros::Publisher radar_points_pub;
        ros::Publisher bbox_pub;
        ros::Publisher text_pub;
        void radar_points_callback(const ars408_msg::RadarPoints::ConstPtr& rps);
};

Radar_points_visualizer::Radar_points_visualizer()
{
    ros::NodeHandle n;
    this->transformed_radar_sub = n.subscribe("/radar/transformed_messages",1,&Radar_points_visualizer::radar_points_callback,this);
    this->radar_points_pub = n.advertise<visualization_msgs::MarkerArray>("/radar/transformed_marker",1);
    this->text_pub = n.advertise<visualization_msgs::MarkerArray>("/radar/texts",1);
    // this->bbox_pub = n.advertise<visualization_msgs::MarkerArray>("/object_marker_array",1);
};

void Radar_points_visualizer::radar_points_callback(const ars408_msg::RadarPoints::ConstPtr& msg)
{
    visualization_msgs::MarkerArray rps_markerArr;
    visualization_msgs::MarkerArray bbox_markerArr;
    visualization_msgs::Marker rps_marker;
    visualization_msgs::Marker bbox_marker;
    visualization_msgs::MarkerArray text_markerArr;
    visualization_msgs::Marker text_marker;

    rps_marker.action = visualization_msgs::Marker::DELETEALL;
    rps_markerArr.markers.push_back(rps_marker);
    this->radar_points_pub.publish(rps_markerArr);
    rps_markerArr.markers.clear();

    text_marker.action = visualization_msgs::Marker::DELETEALL;
    text_markerArr.markers.push_back(text_marker);
    this->text_pub.publish(text_markerArr);
    text_markerArr.markers.clear();


    int ref_id = 0;
    int ref_id2 = 0;
    for(auto it = msg->rps.begin(); it != msg->rps.end(); it++){
        
        rps_marker.header.frame_id = "base_link";
        rps_marker.header.stamp = ros::Time::now();
        rps_marker.id = ref_id;
        rps_marker.ns = "rps";
        rps_marker.type = visualization_msgs::Marker::CYLINDER;
        rps_marker.action = visualization_msgs::Marker::ADD;

        

        geometry_msgs::Pose p;
        geometry_msgs::Quaternion q;
        geometry_msgs::Vector3 g;
        std_msgs::ColorRGBA c;

        g.x = 1;
        g.y = 1;
        g.z = 1.5;
        q.x = 0;
        q.y = 0;
        q.z = 1.0;

        p.position.x = it->distX;
        p.position.y = it->distY;
        p.position.z = 1.0;
        p.orientation = q;

        c.r = 0.0;
        c.g = 0.0;
        c.b = 1.0;
        c.a = 1.0;

        rps_marker.pose = p;
        rps_marker.scale = g;
        rps_marker.color = c;

        rps_markerArr.markers.push_back(rps_marker);

        text_marker.header.frame_id = "base_link";
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "text";
        text_marker.id = ref_id;

        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        
        std::stringstream ss;
        ss << "Strs : " << it->strs << std::endl;
        ss << "Prob : " << it->prob << std::endl;
        ss << "RCS : " << it->rcs << std::endl;
        ss << "ClassT : " << it->classT << std::endl;


        text_marker.text = ss.str(); 
        text_marker.pose = p;
        text_marker.pose.position.x += 1.5;
        text_marker.pose.orientation.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.5;
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 1.0f;
        text_marker.color.a = 1.0;
        
        text_markerArr.markers.push_back(text_marker);

        ref_id++;
    
    }
	this->radar_points_pub.publish(rps_markerArr);
    this->text_pub.publish(text_markerArr);
    // this->bbox_pub.publish(bbox_markerArr);
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "Visual Radar Points");
    
    Radar_points_visualizer rpv;

    ros::Rate rate(60);
    while (ros::ok()) {
	    ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
