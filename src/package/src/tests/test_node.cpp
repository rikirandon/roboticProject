#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;


    ros::ServiceClient client = nh.serviceClient<block_detection::Coordinate>("get_coordinates");

    block_detection::Coordinate srv;
    srv.request.block_id = 1; 

    if (client.call(srv)) // Invia la richiesta di coordinate
    {
        ROS_INFO("Coordinate ricevute: x=%f, y=%f, z=%f", srv.response.x, srv.response.y, srv.response.z);
    }
    else
    {
        ROS_ERROR("Impossibile ottenere le coordinate");
        return 1;
    }

    return 0;
}
