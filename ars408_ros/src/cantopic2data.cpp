#include <iostream>
#include <map>

#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include "ars408_ros/ARS408_CAN.h"
#include "ars408_msg/Test.h"

class radarDriver
{
    public:
        radarDriver();
        ros::Publisher ars408rviz_pub;
    private:
        ros::NodeHandle node_handle;
        ros::Subscriber cantopic_sub;
        // ros::Publisher ars408rviz_pub_;

        void cantopic_callback(const can_msgs::Frame::ConstPtr& msg);
};

radarDriver::radarDriver(): node_handle("~")
{
    cantopic_sub = node_handle.subscribe("/received_messages", 1000, &radarDriver::cantopic_callback, this);
    ars408rviz_pub = node_handle.advertise<ars408_msg::Test>("/testRect", 1);
}

void radarDriver::cantopic_callback(const can_msgs::Frame::ConstPtr& msg)
{
    #pragma region ShowData
    if (msg->is_error)
    {
        std::cout << "E " << std::hex << msg->id << "#" << std::endl;
    }
    else if (msg->is_extended)
    {
        std::cout << "e " << std::hex << msg->id << "#" << std::endl;
    }
    else if (msg->is_rtr)
    {
        std::cout << "R " << std::hex << msg->id << "#" << std::endl;
    }
    else
    {
        std::cout << "s " << std::hex << msg->id << "#";

        for (int i=0; i < msg->dlc; ++i)
        {
            std::cout << " " << std::setfill('0') << std::setw(2) << std::hex << (unsigned int) msg->data[i];
        }
        std::cout << std::endl;
    }
    #pragma endregion

    #pragma region Cluster
    if (msg->id == 0x600)
    {
        ARS408::ClusterStatus cs;

        cs.NofClustersNear = (unsigned int) msg->data[0];
        cs.NofClustersNear = (unsigned int) msg->data[1];

        unsigned int measCounter_raw = (msg->data[2] << 8) | (msg->data[3]);
        cs.MeasCounter = measCounter_raw;

        unsigned int interfaceVersion_raw = msg->data[3] >> 4;
        cs.InterfaceVersion = interfaceVersion_raw;
    }
    else if (msg->id == 0x701)
    {
        ARS408::Cluster clu;
        
        clu.id = (unsigned int) msg->data[0];

        unsigned int distLong_raw = (msg->data[1] << 5) | (msg->data[2] >> 3);
        clu.DistLong = -500.0 + distLong_raw * 0.2;
        
        unsigned int distLat_raw = ((msg->data[2] & 0b00000011) << 8) | (msg->data[3]);
        clu.DistLat = -102.3 + distLat_raw * 0.2;

        unsigned int vrelLong_raw = (msg->data[4] << 8) | (msg->data[5] >> 6);
        clu.VrelLong = -128.0 + vrelLong_raw * 0.25;

        unsigned int vrelLat_raw = ((msg->data[5] & 0b00111111) << 2) | (msg->data[6] >> 5);
        clu.VrelLat = -64.0 + vrelLat_raw * 0.25;

        clu.DynProp = msg->data[6] & 0b00000111;

        unsigned int rcs_raw = (unsigned int) msg->data[7];
        clu.RCS = -64.0 + rcs_raw * 0.5;
    }
    #pragma endregion

    #pragma region Object
    std::map<int, ARS408::Object> objs;

    if (msg->id == 0x60A)
    {
        ARS408::ObjectStatus objs;

        objs.NofOBjects = (unsigned int) msg->data[0];

        unsigned int measCounter_raw = (msg->data[1] << 8) | (msg->data[2]);
        objs.MeasCounter = measCounter_raw;

        unsigned int interfaceVersion_raw = msg->data[3] >> 4;
        objs.InterfaceVersion = interfaceVersion_raw;
    }
    else if (msg->id == 0x60B)
    {
        ARS408::Object obj;
        
        obj.id = (unsigned int) msg->data[0];

        unsigned int distLong_raw = (msg->data[1] << 5) | (msg->data[2] >> 3);
        obj.DistLong = -500.0 + distLong_raw * 0.2;
        
        unsigned int distLat_raw = ((msg->data[2] & 0b00000111) << 8) | (msg->data[3]);
        obj.DistLat = -204.6 + distLat_raw * 0.2;

        unsigned int vrelLong_raw = (msg->data[4] << 8) | (msg->data[5] >> 6);
        obj.VrelLong = -128.0 + vrelLong_raw * 0.25;

        unsigned int vrelLat_raw = ((msg->data[5] & 0b00111111) << 2) | (msg->data[6] >> 5);
        obj.VrelLat = -64.0 + vrelLat_raw * 0.25;

        obj.DynProp = msg->data[6] & 0b00000111;

        unsigned int rcs_raw = (unsigned int) msg->data[7];
        obj.RCS = -64.0 + rcs_raw * 0.5;

        objs[obj.id] = obj;
    }
    else if (msg->id == 0x60C)
    {
        ARS408::Object::Object_quality obj_q;

        obj_q.id = (unsigned int) msg->data[0];
        obj_q.DistLong_rms=(unsigned int)(msg->data[1] >> 3);
        obj_q.DistLong_rms=(unsigned int)((msg->data[1] & 0b00000111) << 2 | msg->data[2] >> 6);
        obj_q.VrelLong_rms=(unsigned int)((msg->data[2]& 0b00111110) >> 1);
        obj_q.VrelLat_rms=(unsigned int)((msg->data[2] & 0b00000001) << 4 | msg->data[3] >> 4);
        obj_q.ArelLong_rms=(unsigned int)(msg->data[3] << 4 | msg->data[4] >> 7);
        obj_q.ArelLat_rms=(unsigned int)((msg->data[4] & 0b01111100) >> 2);
        obj_q.Orientation_rms=(unsigned)((msg->data[4] & 0b00000011) << 3 | msg->data[5]>>5);
        obj_q.ProbOfExist=(unsigned int)((msg->data[6] & 0b11100000) >> 5);
        obj_q.MeasState=(unsigned int)((msg->data[6] & 0b00011100)>>2);

        objs[obj_q.id].object_quality = obj_q;
    }
    else if (msg->id == 0x60D)
    {
        ARS408::Object::object_extented obj_e;

        obj_e.id = (unsigned int) msg->data[0];

        unsigned int ArelLong_raw = (msg->data[1] << 3) | (msg->data[2] >> 5);
        obj_e.ArelLong = -10.0 + ArelLong_raw * 0.01;

        unsigned int ArellLat_raw = ((msg->data[2] & 0b00011111) << 4) | (msg->data[3] >> 4);
        obj_e.ArellLat = -2.5 + ArellLat_raw * 0.01

        obj_e.Class = msg->data[3] & 0b00000111;

        unsigned int OrientationAngle_raw = (msg->data[4] << 2) | (msg->data[5] >> 6);
        obj_e.OrientationAngle = -180.0 + OrientationAngle_raw * 0.4;

        obj_e.Length = (unsigned int) msg->data[6] * 0.2;

        obj_e.Width = (unsigned int) msg->data[7] * 0.2;

        objs[obj_e.id].object_extented = obj_e;
    }
    else if (msg->id == 0x60E)
    {
        ARS408::Object::Object_collision obj_c;

        obj_c.id = (unsigned int) msg->data[0];

        obj_c.CollDetRegionBitfield = (unsigned int) msg->data[1]

        objs[obj_c.id].Object_collision = obj_c;
    }
    #pragma endregion
}

int main(int argc, char** argv)
{
    std::cout << "Start!" << std::endl;

    ros::init(argc, argv, "cantopic2data");
    radarDriver node;
    ros::Rate r(60);

    double x = 0;
    while(ros::ok())
    {
        ars408_msg::Test t;
        t.x = x;
        t.y = 0;
        t.height = 1;
        t.width = 2;
        t.strs = "Hello\nthis world.";
        node.ars408rviz_pub.publish(t);

        x += 0.01;

        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
