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

class BBox_visualizer
{
    public:
        BBox_visualizer();
    private:
        ros::Subscriber Object_sub;
        ros::Publisher BBox_pub;
        void object_callback(const ars408_msg::RadarPoints::ConstPtr& rps);
};

BBox_visualizer::BBox_visualizer()
{  
    ros::NodeHandle n;
    this->Object_sub = n.subscribe("/radar/transformed_messages",1,&BBox_visualizer::object_callback,this);
    this->BBox_pub = n.advertise<visualization_msgs::MarkerArray>("/object_marker_array",1);
};

void BBox_visualizer::object_callback(const ars408_msg::RadarPoints::ConstPtr& rps)
{
    
}


