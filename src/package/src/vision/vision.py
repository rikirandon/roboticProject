#!/usr/bin/env python

import rospy
from motion_planner.srv import coordinates
from motion_planner.srv import coordinatesResponse


block_1 = (0.5, 0.0, 0.3, 0.0, 0.0, 0.0)
block_2 = (0.7, 0.1, -0.3, 0.0, 0.0, 0.0)
block_3 = (0.2, 0.6, 0.0, 0.0, 0.0, 0.0)

blocks = []
blocks.append(block_1)
blocks.append(block_2)
blocks.append(block_3)

final_block_1= (0.1, 0.1, 0.1, 0.0, 0.0, 0.0)
final_block_2= (0.2, 0.2, 0.2, 0.0, 0.0, 0.0)
final_block_3= (0.3, 0.3, 0.3, 0.0, 0.0, 0.0)
final = []
final.append(final_block_1)
final.append(final_block_2)
final.append(final_block_3)

i=0

def vision():
    rospy.init_node('vision')  # Inizializza il nodo ROS
    # Crea un server del servizio chiamato 'coordinates' utilizzando la definizione del tipo di servizio e la funzione di callback
    service = rospy.Service('coordinates', coordinates, handle_coordinates)
    rospy.loginfo("Server del servizio 'coordinates' pronto")
    rospy.spin()  # Mantieni il programma in esecuzione


def handle_coordinates(req):
    # Questa è la funzione di callback che gestisce le richieste del servizio
    rospy.loginfo("Richiesta di coordinate ricevuta")
    robot_coords = transform_vector(blocks[i])
    final_coords= final[i]
    response = coordinatesResponse()
    response.poseStart= robot_coords
    response.poseFinale= final_coords
    if(i==2):
        response.flag = False
    rospy.loginfo("Risposta alle coordinate: x=%f, y=%f, z=%f", response.x, response.y)
    i+=1
    return response

        
def transform_vector(vector):
    m= np.zeros((4, 4))
    m[0, 0] = 1
    m[0, 1] = 0
    m[0, 2] = 0
    m[0, 3] = 0.5

    m[1, 0] = 0
    m[1, 1] = -1
    m[1, 2] = 0
    m[1, 3] = 0,55

    m[2, 0]=0
    m[2, 1] = 0
    m[2, 2] = -1
    m[2, 3] = 0

    m[3, 0] = 0
    m[3, 1] = 0
    m[3, 2] = 0
    m[3, 3] = 1

    homogeneous_vector = np.append(vector, 1)
    transformed_vector = np.dot(m, homogeneous_vector)
    return transformed_vector[:-1]


if __name__ == "__main__":
    vision()

