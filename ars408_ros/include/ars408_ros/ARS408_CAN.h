#pragma once

namespace ARS408
{
    const std::string RadarPowerCfg[]={
        "Standard (default)",
        "-3dB Tx gain,",
        "-6dB Tx gain",
        "-9dB Tx gain"
    };

    const std::string MotionRxState[]={
        "input ok",
        "speed missing",
        "yaw rate missing",
        "speed and yaw rate missing"
    };


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
        int NVMReadStatus;
        int NVMwriteStatus;
        int MaxDistanceCfg;
        int Persistent_Error;
        int Interference;
        int Temperature_Error;
        int Temporary_Error;
        int Voltage_Error;
        int SensorID;
        int SortIndex;
        int RadarPowerCfg;
        int CtrlRelayCfg;
        int OutputTypeCfg;
        int SendQualityCfg;
        int SendExtInfoCfg;
        int MotionRxState;
        int RCS_Threshold;
        int InvalidClusters;
    };

    /* 0x203 */
    struct FilterHeader{
        double NofClusterFilterCfg;
        double NofObjectFilterCfg;
    };

    /* 0x204 */
    struct FilterCfg{
        int Type;
        int Index;
        int Active;
        double Min_Distance;
        double Max_Distance;
    };

    /* 0x408 */
    struct CollDetState{
        int Activation;
        int NofRegions;
        double MinDetectTime;
        int MeasCounter;
    };

    /* 0x402 */
    struct CollDetRegion{
        int WarningLevel;
        int RegionID;
        double Point1X;
        double Point1Y;
        double Point2X;
        double Point2Y;
        int NofObjects;

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
        int DynProp;
        double RCS;

        /* 0x702 */
        struct Cluster_quality
        {
            int id;
            int DistLong_rms;
            int VrelLong_rms;
            int DistLat_rms;
            int Pdh0;
            int VrelLat_rms;
            int AmbigState;
            int InvalidState;
        }cluster_quality;

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
        int DynProp;
        double RCS;

        /* 0x60C */
        struct Object_quality
        {
            int id = -1;
            int DistLong_rms;
            int DistLat_rms;
            int VrelLong_rms;
            int VrelLat_rms;
            int ArelLong_rms;
            int ArelLat_rms;
            int Orientation_rms;
            int ProbOfExist;
            int MeasState;

        } object_quality;

        /* 0x60D */
        struct Object_extended
        {
            int id = -1;
            double ArelLong;
            double ArellLat;
            int Class = 100;
            double OrientationAngle;
            double Length;
            double Width;

        } object_extended;

        /* 0x60E */
        struct Object_collision
        {
            int id = -1;
            int CollDetRegionBitfield;
        } object_collision;
    };
}