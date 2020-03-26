#include <iostream>
#include <sstream>
#include <map>
#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include "ars408_ros/ARS408_CAN.h"
#include "ars408_msg/Test.h"
#include "ars408_msg/Tests.h"


class radarDriver
{
    public:
        radarDriver();
        std::map<int, ARS408::Object> objs_map;
        std::map<int, ARS408::Cluster> clus_map;
        
    private:
        ros::NodeHandle node_handle;
        ros::Subscriber cantopic_sub;
        ros::Publisher ars408rviz_pub;

        void cantopic_callback(const can_msgs::Frame::ConstPtr& msg);
};

radarDriver::radarDriver(): node_handle("~")
{
    cantopic_sub = node_handle.subscribe("/received_messages", 1000, &radarDriver::cantopic_callback, this);
    ars408rviz_pub = node_handle.advertise<ars408_msg::Tests>("/testRects", 10);
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
        cs.NofClustersFar = (unsigned int) msg->data[1];

        unsigned int measCounter_raw = (msg->data[2] << 8) | (msg->data[3]);
        cs.MeasCounter = measCounter_raw;

        unsigned int interfaceVersion_raw = msg->data[3] >> 4;
        cs.InterfaceVersion = interfaceVersion_raw;


        ars408_msg::Tests ts;

        for (auto it = clus_map.begin(); it != clus_map.end(); it++)
        {           
            ars408_msg::Test t;

            t.id = it->second.id;
            t.x = it->second.DistLong;
            t.y = it->second.DistLat;
            t.height = 1;
            t.width = 1;
            t.strs = ARS408::DynProp[it->second.DynProp];

            std::stringstream ss;
            ss.clear();
            ss << t.strs << "\nRCS: " << it->second.RCS;

            t.strs = ss.str();
            ts.tests.push_back(t);
        }
        ars408rviz_pub.publish(ts);
        clus_map.clear();
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

        clus_map[clu.id] = clu;
    }
    #pragma endregion

    #pragma region Object

    if (msg->id == 0x60A)
    {
        ARS408::ObjectStatus objs;

        objs.NofOBjects = (unsigned int) msg->data[0];

        unsigned int measCounter_raw = (msg->data[1] << 8) | (msg->data[2]);
        objs.MeasCounter = measCounter_raw;

        unsigned int interfaceVersion_raw = msg->data[3] >> 4;
        objs.InterfaceVersion = interfaceVersion_raw;

        ars408_msg::Tests ts;

        for (auto it = objs_map.begin(); it != objs_map.end(); it++)
        {           
            ars408_msg::Test t;

            t.id = it->second.id;
            t.x = it->second.DistLong;
            t.y = it->second.DistLat;
            t.height = it->second.object_extented.Length;
            t.width = it->second.object_extented.Width;
            t.angle=it->second.object_extented.OrientationAngle;

            t.classT = int(it->second.object_extented.Class);
            t.strs = ARS408::Class[it->second.object_extented.Class];

            if (it->second.object_quality.ProbOfExist < 4)
                continue;

            t.strs = ARS408::ProbOfExist[it->second.object_quality.ProbOfExist];
            std::stringstream ss;
            ss.clear();
            ss << t.strs << "\nRCS: " << it->second.RCS;

            t.strs = ss.str();
            ts.tests.push_back(t);
        }
        ars408rviz_pub.publish(ts);
        objs_map.clear();
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

        objs_map[obj.id] = obj;
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

        objs_map[obj_q.id].object_quality = obj_q;
    }
    else if (msg->id == 0x60D)
    {
        ARS408::Object::Object_extented obj_e;

        obj_e.id = (unsigned int) msg->data[0];

        unsigned int ArelLong_raw = (msg->data[1] << 3) | (msg->data[2] >> 5);
        obj_e.ArelLong = -10.0 + ArelLong_raw * 0.01;

        unsigned int ArellLat_raw = ((msg->data[2] & 0b00011111) << 4) | (msg->data[3] >> 4);
        obj_e.ArellLat = -2.5 + ArellLat_raw * 0.01;

        obj_e.Class = msg->data[3] & 0b00000111;

        unsigned int OrientationAngle_raw = (msg->data[4] << 2) | (msg->data[5] >> 6);
        obj_e.OrientationAngle = -180.0 + OrientationAngle_raw * 0.4;

        obj_e.Length = (unsigned int) msg->data[6] * 0.2;

        obj_e.Width = (unsigned int) msg->data[7] * 0.2;
        
        objs_map[obj_e.id].object_extented = obj_e;
        std::cout<<obj_e.OrientationAngle<<"\n";
    }
    else if (msg->id == 0x60E)
    {
        ARS408::Object::Object_collision obj_c;

        obj_c.id = (unsigned int) msg->data[0];

        obj_c.CollDetRegionBitfield = (unsigned int) msg->data[1];

        objs_map[obj_c.id].object_collision = obj_c;
    }
    #pragma endregion

    #pragma region radar
    if(msg->id==0x201){
        ARS408::RadarState radar;
        radar.NVMwriteStatus=(unsigned int)(msg->data[0] >> 7);
        radar.NVMReadStatus=(unsigned int)(msg->data[0] & 0b01000000) >> 6;
        radar.MaxDistanceCfg=((unsigned int)(msg->data[1] << 2 | msg->data[1] >> 6))*2;
        radar.Persistent_Error=(unsigned int)(msg->data[2] & 0b00100000) >> 5;
        radar.Interference=(unsigned int)(msg->data[2] & 0b00010000) >> 4;
        radar.Temperature_Error=(unsigned int)(msg->data[2] & 0b00001000) >> 3;
        radar.Temporary_Error=(unsigned int)(msg->data[2] & 0b00000100) >> 2;
        radar.Persistent_Error=(unsigned int)(msg->data[2] & 0b00000010) >> 1;
        radar.RadarPowerCfg=(unsigned int)((msg->data[3] & 0b00000011) << 1) | (msg->data[4] >> 7);
        radar.SortIndex=(unsigned int)(msg->data[4] & 0b01110000) >> 4;
        radar.SensorID=(unsigned int)(msg->data[4] & 0b00000111);
        radar.MotionRxState=(unsigned int)(msg->data[5] >> 6);
        radar.SendQualityCfg=(unsigned int)((msg->data[5] & 0b00010000) >> 4);
        radar.OutputTypeCfg=(unsigned int)((msg->data[5] & 0b00001100) >> 2);
        radar.CtrlRelayCfg=(unsigned int)((msg->data[5] & 0b00000010) >> 1);
        radar.InvalidClusters=(unsigned int)(msg->data[6]);
        radar.RCS_Threshold=(unsigned int)((msg->data[7] & 0b00011100)>>2);
    }
    #pragma endregion

    #pragma region filter
    if(msg->id==0x203){
        ARS408::FilterHeader fil_h;
        fil_h.NofClusterFilterCfg=(unsigned int)(msg->data[0] >> 3);
        fil_h.NofObjectFilterCfg=(unsigned int)(msg->data[1] >> 3);
    }
    else if(msg->id==0x204){
        ARS408::FilterCfg fil_c;
        fil_c.Type=(unsigned int)(msg->data[0]>>7);
        fil_c.Index=(unsigned int)((msg->data[0] & 0b01111000) >> 3);
        fil_c.Min_Distance=(unsigned int)(msg->data[1] << 8 | msg->data[2]);
        fil_c.Max_Distance=(unsigned int)(msg->data[3] << 8 | msg->data[4]);
    }
    #pragma endregion

    #pragma region collision
    if(msg->id==0x408){
        ARS408::CollDetState col;
        col.NofRegions=(unsigned int)(msg->data[0] >> 4);
        col.Activation=(unsigned int)((msg->data[0] &0b00000010)>>1);
        col.MinDetectTime=((unsigned int)(msg->data[1]))*0.1;
        col.MeasCounter=(unsigned int)(msg->data[2] << 8 | msg->data[3]);
    }
    else if(msg->id==0x402){
        ARS408::CollDetRegion col_d;
        col_d.RegionID=(unsigned int)(msg->data[0] >> 5);
        col_d.WarningLevel=(unsigned int)((msg->data[0] & 0b00011000)>>3);
        
        unsigned int point1_x=msg->data[1] << 5 | msg->data[2] >>3;
        col_d.Point1X = -500.0 + point1_x * 0.2;

        unsigned int point1_y=msg->data[2] << 8 | msg->data[3];
        col_d.Point1Y =-204.6 + point1_y * 0.2;
        
        unsigned int point2_x=msg->data[4] << 5 | msg->data[5] >>3;
        col_d.Point2X = -500.0 + point2_x * 0.2;

        unsigned int point2_y=msg->data[5] << 8 | msg->data[6];
        col_d.Point2Y = -204.6 + point2_y * 0.2;

        col_d.NofObjects=msg->data[7]; 
    }
    #pragma endregion
}

int main(int argc, char** argv)
{
    std::cout << "Start!" << std::endl;

    ros::init(argc, argv, "cantopic2data");
    radarDriver node;
    ros::Rate r(60);
    // ars408_msg::Tests ts;
    while(ros::ok())
    {
        // ars408_msg::Test t;
        // t.width=2;
        // t.height=1;
        // t.angle=-0.5;
        // t.x=0;
        // t.y=0;
        // ts.tests.push_back(t);
        // node.ars408rviz_pub.publish(ts);
        ros::spinOnce();
        r.sleep();
    }
    
    return 0;
}
