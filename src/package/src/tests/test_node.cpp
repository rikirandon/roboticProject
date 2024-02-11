#include <ros/ros.h>
#include <roboticProject/coordinate.h> 
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Simulazione della chiamata del servizio al nodo vision
    ros::ServiceClient client = nh.serviceClient<coordinate::Coordinate>("vision");

    // Creazione di un oggetto Coordinate
    coordinate::poseStart srv;
   
    // Chiamata al servizio
    if (client.call(srv)) {
        ROS_INFO("Chiamata al servizio avvenuta con successo");
        // Stampa delle coordinate acquisite
        std::cout << "Coordinate acquisite: x = " << srv.response.poseStart<< std::endl;
    } else {
        ROS_ERROR("Errore durante la chiamata al servizio");
        return 1;
    }

    return 0;
}
