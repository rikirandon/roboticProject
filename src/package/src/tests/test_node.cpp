#include <ros/ros.h>
#include "package/coordinates.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Simulazione della chiamata del servizio al nodo vision
    ros::ServiceClient client = nh.serviceClient<package::coordinates>("coordinates");
    package::coordinates srv;
    // Chiamate successive fino a quando la lista di blocchi non è vuota
    while (ros::ok()) {
    	
        // Chiamata al servizio
        if (client.call(srv)) {
            ROS_INFO("Chiamata al servizio avvenuta con successo");
            // Stampa delle coordinate acquisite
            geometry_msgs::Pose poseStart = srv.response.poseStart;
            geometry_msgs::Pose poseFinal = srv.response.poseFinal;
            std::cout << "Coordinate poseStart: x = " << poseStart.position.x << ", y = " << poseStart.position.y << ", z = " <<poseStart.position.z<< std::endl;
            std::cout << "Coordinate poseFInal: x = " << poseFinal.position.x << ", y = " << poseFinal.position.y << ", z = " <<poseFinal.position.z<< std::endl;
        } else {
            ROS_ERROR("Errore durante la chiamata al servizio");
            return 1;
        }

        // Controllo se la lista di blocchi è vuota
        if (!srv.response.flag) {
            ROS_INFO("Lista di blocchi vuota. Terminazione del programma.");
            break; // Esci dal ciclo while
        }

        // Aggiorna la frequenza di chiamata al servizio, se necessario
        ros::Duration(5.0).sleep(); // Attendi 1 secondo prima della prossima chiamata
    }

    return 0;
}
