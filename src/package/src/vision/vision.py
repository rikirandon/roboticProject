#!/usr/bin/env python

import rospy
from motion_planner.srv import coordinates
from motion_planner.srv import coordinatesResponse

def handle_coordinates(req):
    # Questa è la funzione di callback che gestisce le richieste del servizio
    rospy.loginfo("Richiesta di coordinate ricevuta: x=%f, y=%f", req.x, req.y)
    # Qui puoi inserire la logica per elaborare le coordinate ricevute
    # Ad esempio, potresti fare qualche calcolo e restituire una risposta
    response = coordinatesResponse()
    response.flag = True
    #rospy.loginfo("Risposta alle coordinate: x=%f, y=%f", response.x, response.y)
    return response

def vision():
    rospy.init_node('vision')  # Inizializza il nodo ROS
    # Crea un server del servizio chiamato 'coordinates' utilizzando la definizione del tipo di servizio e la funzione di callback
    service = rospy.Service('coordinates', coordinates, handle_coordinates)
    rospy.loginfo("Server del servizio 'coordinates' pronto")
    rospy.spin()  # Mantieni il programma in esecuzione

if __name__ == "__main__":
    vision()

