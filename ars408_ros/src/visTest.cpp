#include <string>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "ars408_ros/ARS408_CAN.h"
#include "ars408_msg/Test.h"

class visDriver
{
    public:
        visDriver();
    private:
        ros::NodeHandle node_handle;
        ros::Subscriber ars408rviz_sub;
        ros::Publisher marker_pub;

        void ars408rviz_callback(const ars408_msg::Test::ConstPtr& msg);
};


visDriver::visDriver()
{
    node_handle = ros::NodeHandle("~");
    ars408rviz_sub = node_handle.subscribe("/testRect", 1, &visDriver::ars408rviz_callback, this);
    marker_pub = node_handle.advertise<visualization_msgs::Marker>("markers", 2);
}

void visDriver::ars408rviz_callback(const ars408_msg::Test::ConstPtr& msg)
{
    // Rect
    visualization_msgs::Marker marker_rect;
    marker_rect.header.frame_id = "/my_frame";
    marker_rect.header.stamp = ros::Time::now();

    marker_rect.ns = "rect";
    marker_rect.id = 0;

    marker_rect.type = visualization_msgs::Marker::CUBE;
    marker_rect.action = visualization_msgs::Marker::ADD;

    marker_rect.pose.position.x = msg->x;
    marker_rect.pose.position.y = msg->y;
    marker_rect.pose.position.z = msg->height / 2.0;
    marker_rect.pose.orientation.x = 0.0;
    marker_rect.pose.orientation.y = 0.0;
    marker_rect.pose.orientation.z = 0.0;
    marker_rect.pose.orientation.w = 1.0;
    marker_rect.scale.x = 0.1;
    marker_rect.scale.y = msg->width;
    marker_rect.scale.z = msg->height;

    marker_rect.color.r = 1.0f;
    marker_rect.color.g = 0.0f;
    marker_rect.color.b = 0.0f;
    marker_rect.color.a = 1.0;

    marker_rect.lifetime = ros::Duration(1);

    // Text
    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = "/my_frame";
    marker_text.header.stamp = ros::Time::now();

    marker_text.ns = "text";
    marker_text.id = 1;

    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.text = msg->strs;

    marker_text.pose.position.x = msg->x;
    marker_text.pose.position.y = msg->y;
    marker_text.pose.position.z = msg->height + 0.4;
    marker_text.pose.orientation.x = 0.0;
    marker_text.pose.orientation.y = 0.0;
    marker_text.pose.orientation.z = 0.0;
    marker_text.pose.orientation.w = 1.0;
    marker_text.scale.z = 0.2;

    marker_text.color.r = 1.0f;
    marker_text.color.g = 1.0f;
    marker_text.color.b = 1.0f;
    marker_text.color.a = 1.0;

    marker_text.lifetime = ros::Duration(1);

    // Publish
    marker_pub.publish(marker_rect);
    marker_pub.publish(marker_text);
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