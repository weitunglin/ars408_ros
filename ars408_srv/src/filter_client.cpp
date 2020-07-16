#include "ros/ros.h"
#include "ars408_srv/Filter.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ars408_srv::Filter>("/filter");
    ars408_srv::Filter srv;

    srv.request.RCS_filter = std::stof(argv[1]);

    if (client.call(srv))
    {
        std::cout << "RCS_filter: " << srv.response.RCS_filter << std::endl;
    }
    else
    {
        std::cout << "Failed to call service filter\n";
        return 1;
    }

    return 0;
}