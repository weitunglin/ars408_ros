#include <string>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "ars408_ros/ARS408_CAN.h"
#include "ars408_msg/Test.h"
#include "ars408_msg/Tests.h"

class visDriver
{
    public:
        visDriver();
    private:
        ros::NodeHandle node_handle;
        ros::Subscriber ars408rviz_sub;
        ros::Publisher markerArr_pub;

        void ars408rviz_callback(const ars408_msg::Tests::ConstPtr& msg);
};


visDriver::visDriver()
{
    node_handle = ros::NodeHandle("~");
    ars408rviz_sub = node_handle.subscribe("/testRects", 10, &visDriver::ars408rviz_callback, this);
    markerArr_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/markersArr", 10);
}

void visDriver::ars408rviz_callback(const ars408_msg::Tests::ConstPtr& msg)
{
    visualization_msgs::MarkerArray marArr;
    
    for (auto it = msg->tests.begin(); it != msg->tests.end(); ++it)
    {        
        // Rect
        visualization_msgs::Marker marker_rect;
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
        marker_rect.pose.orientation.z = it->angle;
        marker_rect.pose.orientation.w = 1.0;
        marker_rect.scale.x = it->height;
        marker_rect.scale.y = it->width;
        marker_rect.scale.z = 0.1;

        if (it->classT == 0x00)
        {
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x01)
        {
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x02)
        {
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x04)
        {
            marker_rect.color.r = 1.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x05)
        {
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 0.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x06)
        {
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 1.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }
        else if (it->classT == 0x07 || it->classT==0x03)
        {
            marker_rect.color.r = 0.0f;
            marker_rect.color.g = 0.0f;
            marker_rect.color.b = 1.0f;
            marker_rect.color.a = 1.0;
        }
        else
        {
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
        marker_text.text = it->strs;

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

        marArr.markers.push_back(marker_rect);
        marArr.markers.push_back(marker_text);
    }
    
    if (msg->tests.size() > 0)
        markerArr_pub.publish(marArr);
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "visTest");
    visDriver node;
    ros::Rate r(60);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}