#include <ros/ros.h>
#include <roboticProject/coordinate.h> 
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    // Simulazione della chiamata del servizio al nodo vision
    ros::ServiceClient client = nh.serviceClient<coordinate::Coordinate>("vision");

    // Chiamate successive fino a quando la lista di blocchi non è vuota
    while (ros::ok()) {
        // Creazione di un oggetto Coordinate per inviarlo al nodo vision
        coordinate::poseStart srv;

        // Chiamata al servizio
        if (client.call(srv)) {
            ROS_INFO("Chiamata al servizio avvenuta con successo");
            // Stampa delle coordinate acquisite
            std::cout << "Coordinate acquisite: x = " << srv.response.x << ", y = " << srv.response.y << std::endl;
        } else {
            ROS_ERROR("Errore durante la chiamata al servizio");
            return 1;
        }

         // Controllo se la lista di blocchi è vuota
        if (srv.response.block_list.empty()) {
            ROS_INFO("Lista di blocchi vuota. Terminazione del programma.");
            break; // Esci dal ciclo while
        }

        // Aggiorna la frequenza di chiamata al servizio, se necessario
        ros::Duration(5.0).sleep(); // Attendi 1 secondo prima della prossima chiamata
    }

    return 0;
}