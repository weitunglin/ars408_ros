#pragma once

namespace ARS408
{
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
    };
}