#pragma once

namespace ARS408
{
    const std::string DynProp[]={
        "moving",
        "stationary",
        "oncoming",
        "crossing left",
        "crossing right",
        "unknown",
        "stopped",
    };

    const std::string Class[]={
        "point",
        "car",
        "truck",
        "reserved",
        "motorcycle",
        "bicycle",
        "wide",
        "reserved"
    };

    const std::string ProbOfExist[]={
        "invalid",
        "<25%",
        "<50%",
        "<75%",
        "<90%",
        "<99%",
        "<99.9%",
        "<=100%",
    };

    /* 0x201 */
    struct RadarState
    {
        double NVMReadStatus;
        double NVMwriteStatus;
        double MaxDistanceCfg;
        double Persistent_Error;
        double Interference;
        double Temperature_Error;
        double Temporary_Error;
        double Voltage_Error;
        double SensorID;
        double SortIndex;
        double RadarPowerCfg;
        double CtrlRelayCfg;
        double OutputTypeCfg;
        double SendQualityCfg;
        double SendExtInfoCfg;
        double MotionRxState;
        double RCS_Threshold;
        double InvalidClusters;
    };

    /* 0x203 */
    struct FilterHeader{
        double NofClusterFilterCfg;
        double NofObjectFilterCfg;
    };

    /* 0x204 */
    struct FilterCfg{
        double Type;
        double Index;
        double Active;
        double Min_Distance;
        double Max_Distance;

    };
    
    /* 0x408 */
    struct CollDetState{
        double Activation;
        double NofRegions;
        double MinDetectTime;
        double MeasCounter;
    };

    /* 0x402 */
    struct CollDetRegion{
        double WarningLevel;
        double RegionID;
        double Point1X;
        double Point1Y;
        double Point2X;
        double Point2Y;
        double NofObjects;
        
    };
    

    /* 0x600 */
    struct ClusterStatus
    {
        int NofClustersNear;
        int NofClustersFar;
        int MeasCounter;
        int InterfaceVersion;
    };

    /* 0x701 */
    struct Cluster
    {
        int id;
        double DistLong;
        double DistLat;
        double VrelLong;
        double VrelLat;
        double DynProp;
        double RCS;
    };

    /* 0x60A */
    struct ObjectStatus
    {
        int NofOBjects;
        int MeasCounter;
        int InterfaceVersion;
    };

    /* 0x60B */
    struct Object
    {
        int id;
        double DistLong;
        double DistLat;
        double VrelLong;
        double VrelLat;
        double DynProp;
        double RCS;
    
        /* 0x60C */
        struct Object_quality
        {
            int id;
            double DistLong_rms;
            double DistLat_rms;
            double VrelLong_rms;
            double VrelLat_rms;
            double ArelLong_rms;
            double ArelLat_rms;
            double Orientation_rms;
            double ProbOfExist;
            double MeasState;

        } object_quality;

        /* 0x60D */
        struct Object_extented
        {
            int id;
            double ArelLong;
            double ArellLat;
            double Class;
            double OrientationAngle;
            double Length;
            double Width;

        } object_extented;

        /* 0x60E */
        struct Object_collision
        {
            int id;
            double CollDetRegionBitfield;
                        
        } object_collision;
    };
}