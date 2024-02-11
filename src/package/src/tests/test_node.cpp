#include <ros/ros.h>
#include <package/coordinates.h> 
#include "std_msgs/Float64MultiArray.h"
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Simulazione della chiamata del servizio al nodo vision
    ros::ServiceClient client = nh.serviceClient<package::coordinates>("coordinates");
    package::coordinates srv;
    package::coordinates::Request req;
    // Chiamate successive fino a quando la lista di blocchi non è vuota
    while (ros::ok()) {
    	
        // Chiamata al servizio
        if (client.call(req, srv)) {
            ROS_INFO("Chiamata al servizio avvenuta con successo");
            // Stampa delle coordinate acquisite
            std_msgs::Float64MultiArray poseStart = srv.response.poseStart;
            std::cout << "Coordinate acquisite: x = " << poseStart.data[0] << ", y = " << poseStart.data[1] << ", z = " <<poseStart.data[2]<< std::endl;
        } else {
            ROS_ERROR("Errore durante la chiamata al servizio");
            return 1;
        }

<<<<<<< HEAD
        // Controllo se la lista di blocchi è vuota
        if (!srv.response.flag) {
=======
         // Controllo se la lista di blocchi è vuota
        if (srv.response.block_list.empty()) {
>>>>>>> f4a928a5c9617e064583fbe306a5cc774eaafa31
            ROS_INFO("Lista di blocchi vuota. Terminazione del programma.");
            break; // Esci dal ciclo while
        }

        // Aggiorna la frequenza di chiamata al servizio, se necessario
        ros::Duration(5.0).sleep(); // Attendi 1 secondo prima della prossima chiamata
    }

    return 0;
}
